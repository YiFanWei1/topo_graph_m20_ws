#include "nav2_route3d/pcd_io.hpp"

// 本文件提供骨架构图所需的最小 PCD 读取能力：
// 1. 解析 PCD 头部，定位 x/y/z 字段；
// 2. 支持 DATA ascii、binary 和 binary_compressed 三种常见存储形式；
// 3. binary_compressed 使用 PCD/PCL 采用的 LZF 算法解压；
// 4. 按若干常见命名规则查找“位姿索引对应的 PCD 文件”。
//
// 返回值只包含 XYZ，不保留 intensity、ring、time、rgb 等其他字段。

#include <algorithm>
// snprintf 用来产生 6 位和 8 位补零文件名。
#include <cstdio>
// memcpy 用于从二进制字节数组安全复制标量，并用于 LZF 块拷贝。
#include <cstring>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace nav2_route3d
{
namespace
{

// 按任意空白字符切分一行文本。
// stringstream 的 >> 会自动折叠连续空格和 Tab，适合解析 PCD 头部及 ASCII 点数据。
std::vector<std::string> split(const std::string & line)
{
  std::stringstream stream(line);
  std::vector<std::string> result;
  std::string item;
  while (stream >> item) {
    result.push_back(item);
  }
  return result;
}

// 从字节数组的指定偏移读取一个 T 类型标量。
// 使用 memcpy 而不是 reinterpret_cast，避免未对齐地址访问和严格别名问题。
template<typename T>
T readScalar(const std::vector<uint8_t> & data, const size_t offset)
{
  // 在真正复制前检查完整的 sizeof(T) 字节仍位于 payload 内。
  if (offset + sizeof(T) > data.size()) {
    throw std::runtime_error("PCD binary payload ended unexpectedly");
  }
  T value;
  // PCD 二进制数据会按本机标量布局直接解释；这里没有做大小端转换。
  std::memcpy(&value, data.data() + offset, sizeof(T));
  return value;
}

// binary_compressed 数据区开头依次存放两个 uint32：压缩后大小和解压后大小。
uint32_t readUint32(std::istream & input)
{
  uint32_t value = 0U;
  input.read(reinterpret_cast<char *>(&value), sizeof(value));
  // 读取不足 4 字节时，说明压缩数据头部损坏或文件被截断。
  if (!input.good()) {
    throw std::runtime_error("PCD binary_compressed header ended unexpectedly");
  }
  return value;
}

// 解压 PCD binary_compressed 使用的 LZF 数据。
// LZF 数据由两类指令组成：短控制字表示“原样字节串”，长控制字表示“向后引用”。
std::vector<uint8_t> decompressLzf(
  const std::vector<uint8_t> & compressed,
  const size_t uncompressed_size)
{
  // 输出大小来自 PCD 压缩块头部；解压过程中所有写入都必须落在此范围内。
  std::vector<uint8_t> output(uncompressed_size);
  // ip：压缩输入读取位置；op：解压输出写入位置。
  size_t ip = 0U;
  size_t op = 0U;

  while (ip < compressed.size()) {
    // 每条 LZF 指令以一个控制字节开始。
    const uint8_t ctrl = compressed[ip++];
    if (ctrl < 32U) {
      // 高 3 位全为 0 时是 literal run；实际长度为 ctrl+1，即 1~32 字节。
      const size_t length = static_cast<size_t>(ctrl) + 1U;
      // 同时保护输入不越界和输出不越界。
      if (ip + length > compressed.size() || op + length > output.size()) {
        throw std::runtime_error("Invalid LZF literal run in PCD binary_compressed payload");
      }
      // literal run 不做转换，直接从压缩流复制到输出流。
      std::memcpy(output.data() + op, compressed.data() + ip, length);
      ip += length;
      op += length;
      continue;
    }

    // back-reference：控制字高 3 位编码基础长度，低 5 位编码距离的高位。
    size_t length = static_cast<size_t>(ctrl >> 5U);
    size_t reference_offset = static_cast<size_t>(ctrl & 0x1fU) << 8U;
    if (length == 7U) {
      // 基础长度达到 7 时，再从流中读取一个字节作为扩展长度。
      if (ip >= compressed.size()) {
        throw std::runtime_error("Invalid LZF back-reference length in PCD binary_compressed payload");
      }
      length += compressed[ip++];
    }
    // 距离的低 8 位紧跟在控制字/扩展长度之后。
    if (ip >= compressed.size()) {
      throw std::runtime_error("Invalid LZF back-reference offset in PCD binary_compressed payload");
    }
    reference_offset += compressed[ip++];
    // LZF 规范对回引用匹配长度额外加 2。
    length += 2U;

    // 引用位置必须已经存在于输出中，且本次展开不能超过预期输出大小。
    if (reference_offset + 1U > op || op + length > output.size()) {
      throw std::runtime_error("Invalid LZF back-reference in PCD binary_compressed payload");
    }
    // 实际引用起点为当前位置向前 reference_offset+1 个字节。
    size_t ref = op - reference_offset - 1U;
    // 逐字节复制是有意的：LZF 允许引用区间与当前输出区间重叠，
    // 新写出的字节可以继续作为后续字节的引用来源。
    for (size_t i = 0; i < length; ++i) {
      output[op++] = output[ref++];
    }
  }

  // 输入耗尽后，输出字节数必须与压缩头声明的解压大小完全一致。
  if (op != output.size()) {
    throw std::runtime_error("LZF decompressed PCD size mismatch");
  }
  return output;
}

// 根据 PCD 头部的 TYPE 和 SIZE，把一个二进制字段统一转换为 double。
double readFieldAsDouble(
  const std::vector<uint8_t> & data,
  const size_t offset,
  const size_t size,
  const std::string & type)
{
  // TYPE=F：IEEE 浮点数，支持 4 字节 float 和 8 字节 double。
  if (type == "F" && size == 4U) {
    return static_cast<double>(readScalar<float>(data, offset));
  }
  if (type == "F" && size == 8U) {
    return readScalar<double>(data, offset);
  }
  // TYPE=I：有符号整数，支持 8/16/32 位。
  if (type == "I" && size == 1U) {
    return static_cast<double>(readScalar<int8_t>(data, offset));
  }
  if (type == "I" && size == 2U) {
    return static_cast<double>(readScalar<int16_t>(data, offset));
  }
  if (type == "I" && size == 4U) {
    return static_cast<double>(readScalar<int32_t>(data, offset));
  }
  // TYPE=U：无符号整数，支持 8/16/32 位。
  if (type == "U" && size == 1U) {
    return static_cast<double>(readScalar<uint8_t>(data, offset));
  }
  if (type == "U" && size == 2U) {
    return static_cast<double>(readScalar<uint16_t>(data, offset));
  }
  if (type == "U" && size == 4U) {
    return static_cast<double>(readScalar<uint32_t>(data, offset));
  }
  // 例如 64 位整数或头部给出的非法组合不会被静默误读，而是明确报错。
  throw std::runtime_error("Unsupported PCD field type/size: " + type + "/" + std::to_string(size));
}

}  // namespace

// 读取一个 PCD 文件，并按文件中 FIELDS 的实际位置提取每个点的 x、y、z。
std::vector<Point3D> readPcdXYZ(const std::filesystem::path & path)
{
  // 即使是 ASCII PCD 也用 binary 模式打开，确保 tellg/seekg 和二进制 payload 的位置一致。
  std::ifstream input(path, std::ios::binary);
  if (!input.good()) {
    throw std::runtime_error("Cannot open PCD file: " + path.string());
  }

  // header 的键是 FIELDS、SIZE、TYPE、COUNT 等，值是不包含键名的其余 token。
  // unordered_map 足够使用，因为后续按键查询，不依赖头部原始顺序。
  std::unordered_map<std::string, std::vector<std::string>> header;
  std::string line;
  // 默认值只作为解析兜底；规范 PCD 应明确包含 DATA 行。
  std::string data_kind = "ascii";
  std::streampos data_start{};
  while (std::getline(input, line)) {
    // 跳过空行以及以 # 开头的注释行。
    if (line.empty() || line[0] == '#') {
      continue;
    }
    const auto parts = split(line);
    if (parts.empty()) {
      continue;
    }
    if (parts[0] == "DATA") {
      // DATA 后的值通常是 ascii、binary 或 binary_compressed。
      if (parts.size() >= 2U) {
        data_kind = parts[1];
      }
      // getline 已消费 DATA 行末尾的换行符，此位置就是实际数据区的起点。
      data_start = input.tellg();
      break;
    }
    // 同名头部键若重复出现，保留最后一次的内容。
    header[parts[0]] = std::vector<std::string>(parts.begin() + 1, parts.end());
  }

  // 没有 FIELDS 就无法判断每列的语义，直接拒绝读取。
  if (header.count("FIELDS") == 0U) {
    throw std::runtime_error("PCD missing FIELDS: " + path.string());
  }
  const auto & fields = header.at("FIELDS");
  // 返回指定字段在 FIELDS 数组中的编号；缺少任意 XYZ 都不能用于骨架障碍检测。
  const auto find_field = [&fields](const std::string & name) {
      const auto it = std::find(fields.begin(), fields.end(), name);
      if (it == fields.end()) {
        throw std::runtime_error("PCD missing field: " + name);
      }
      return static_cast<size_t>(std::distance(fields.begin(), it));
    };
  const auto x_idx = find_field("x");
  const auto y_idx = find_field("y");
  const auto z_idx = find_field("z");

  std::vector<Point3D> points;
  if (data_kind == "ascii") {
    // ASCII 模式每一行通常对应一个点，每个 token 对应 FIELDS 中的一个标量字段。
    while (std::getline(input, line)) {
      const auto parts = split(line);
      // 空行或列数不足的坏行被跳过，不影响文件中其他合法点。
      if (parts.size() <= std::max({x_idx, y_idx, z_idx})) {
        continue;
      }
      // stod 同时支持普通小数、科学计数法、nan 和 inf；非法数值会抛出异常。
      points.push_back(
        {std::stod(parts[x_idx]), std::stod(parts[y_idx]), std::stod(parts[z_idx])});
    }
    return points;
  }

  // 二进制模式需要 SIZE/TYPE/COUNT 才能计算每个字段的字节布局。
  // SIZE 是必需项，使用 at() 会在缺失时直接抛出异常。
  const auto sizes_raw = header.at("SIZE");
  // TYPE 缺失时按 PCD 常见默认值 F 处理。
  const auto types = header.count("TYPE") != 0U ?
    header.at("TYPE") : std::vector<std::string>(fields.size(), "F");
  // COUNT 缺失时每个字段默认只有一个标量。
  const auto counts_raw = header.count("COUNT") != 0U ?
    header.at("COUNT") : std::vector<std::string>(fields.size(), "1");
  // 优先采用 POINTS；缺失时退化为 WIDTH。
  // 该代码面向常见的一维点云文件，若只提供 WIDTH/HEIGHT，不会再额外乘 HEIGHT。
  const size_t point_count = header.count("POINTS") != 0U ?
    static_cast<size_t>(std::stoul(header.at("POINTS")[0])) :
    static_cast<size_t>(std::stoul(header.at("WIDTH")[0]));

  // sizes[i]：字段中单个标量的字节数；counts[i]：一个点内该字段包含的标量数；
  // offsets[i]：在普通 binary 的单点记录中，该字段相对于点起始位置的字节偏移。
  std::vector<size_t> sizes(fields.size(), 0U);
  std::vector<size_t> counts(fields.size(), 1U);
  std::vector<size_t> offsets(fields.size(), 0U);
  // point_step 是普通 binary 模式下一个完整点记录占用的总字节数。
  size_t point_step = 0U;
  for (size_t i = 0; i < fields.size(); ++i) {
    sizes[i] = static_cast<size_t>(std::stoul(sizes_raw[i]));
    counts[i] = static_cast<size_t>(std::stoul(counts_raw[i]));
    offsets[i] = point_step;
    point_step += sizes[i] * counts[i];
  }

  // 头部解析完成后，显式回到 DATA 行之后记录的数据起点。
  input.seekg(data_start);
  if (data_kind == "binary") {
    // 普通 binary 使用 AoS（Array of Structures）布局：
    // [point0 的全部字段][point1 的全部字段]...[pointN 的全部字段]。
    std::vector<uint8_t> binary(point_step * point_count);
    input.read(reinterpret_cast<char *>(binary.data()), static_cast<std::streamsize>(binary.size()));
    // gcount 必须等于头部计算出的理论 payload 大小，否则文件不完整。
    if (static_cast<size_t>(input.gcount()) != binary.size()) {
      throw std::runtime_error("PCD binary payload shorter than expected: " + path.string());
    }
    points.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
      // base 指向第 i 个点记录的首字节，再叠加字段偏移即可读取 XYZ。
      const auto base = i * point_step;
      points.push_back(
        {
          readFieldAsDouble(binary, base + offsets[x_idx], sizes[x_idx], types[x_idx]),
          readFieldAsDouble(binary, base + offsets[y_idx], sizes[y_idx], types[y_idx]),
          readFieldAsDouble(binary, base + offsets[z_idx], sizes[z_idx], types[z_idx])
        });
    }
    return points;
  }

  if (data_kind == "binary_compressed") {
    // PCD binary_compressed payload 的前 8 字节不是点数据：
    // 先是 uint32 compressed_size，再是 uint32 uncompressed_size。
    const uint32_t compressed_size = readUint32(input);
    const uint32_t uncompressed_size = readUint32(input);
    // 严格按照头部声明读取压缩块。
    std::vector<uint8_t> compressed(compressed_size);
    input.read(
      reinterpret_cast<char *>(compressed.data()),
      static_cast<std::streamsize>(compressed.size()));
    // 实际读取字节不足表示压缩块被截断。
    if (static_cast<size_t>(input.gcount()) != compressed.size()) {
      throw std::runtime_error("PCD binary_compressed payload shorter than expected: " + path.string());
    }
    // LZF 解压函数还会检查每一条 literal/back-reference 指令是否越界。
    const auto binary = decompressLzf(compressed, uncompressed_size);
    // PCD 头部算出的所有点字段总字节数必须和解压结果一致。
    if (binary.size() != point_step * point_count) {
      throw std::runtime_error("PCD binary_compressed decompressed size does not match header: " + path.string());
    }

    // binary_compressed 解压后的布局与普通 binary 不同，它采用 SoA/字段优先布局：
    // [全部点的 field0][全部点的 field1]...[全部点的 fieldN]。
    // compressed_offsets[i] 记录第 i 个字段整块数据的起点。
    std::vector<size_t> compressed_offsets(fields.size(), 0U);
    size_t compressed_offset = 0U;
    for (size_t i = 0; i < fields.size(); ++i) {
      compressed_offsets[i] = compressed_offset;
      // 一个字段块大小 = 单标量字节数 * COUNT * 总点数。
      compressed_offset += sizes[i] * counts[i] * point_count;
    }

    points.reserve(point_count);
    for (size_t i = 0; i < point_count; ++i) {
      // 在各自字段块内移动 i * field_step，分别取出第 i 个点的 XYZ。
      // 若某个字段 COUNT>1，本读取器只取该字段的第一个标量；常规 XYZ 的 COUNT 均为 1。
      points.push_back(
        {
          readFieldAsDouble(
            binary, compressed_offsets[x_idx] + i * sizes[x_idx] * counts[x_idx],
            sizes[x_idx], types[x_idx]),
          readFieldAsDouble(
            binary, compressed_offsets[y_idx] + i * sizes[y_idx] * counts[y_idx],
            sizes[y_idx], types[y_idx]),
          readFieldAsDouble(
            binary, compressed_offsets[z_idx] + i * sizes[z_idx] * counts[z_idx],
            sizes[z_idx], types[z_idx])
        });
    }
    return points;
  }

  // 任何未实现的 DATA 编码都显式报错，避免把未知格式当成空点云。
  throw std::runtime_error("Unsupported PCD DATA type: " + data_kind);
}

// 按轨迹/位姿索引查找对应 PCD 文件，返回第一个存在的候选路径。
std::optional<std::filesystem::path> findPcdForIndex(
  const std::filesystem::path & pcd_dir,
  const size_t index)
{
  // raw 是不补零的十进制编号，例如 12。
  const auto raw = std::to_string(index);
  char buffer6[32];
  char buffer8[32];
  // 同时生成常见的 6 位、8 位前导零编号，例如 000012 和 00000012。
  std::snprintf(buffer6, sizeof(buffer6), "%06zu", index);
  std::snprintf(buffer8, sizeof(buffer8), "%08zu", index);
  // 候选顺序就是匹配优先级：裸编号优先，其次补零编号，最后是 pose_ 前缀形式。
  const std::vector<std::filesystem::path> candidates = {
    pcd_dir / (raw + ".pcd"),
    pcd_dir / (std::string(buffer6) + ".pcd"),
    pcd_dir / (std::string(buffer8) + ".pcd"),
    pcd_dir / ("pose_" + raw + ".pcd"),
    pcd_dir / ("pose_" + std::string(buffer6) + ".pcd")
  };
  for (const auto & candidate : candidates) {
    // 这里只检查路径存在，不在此处验证它是否为普通文件或 PCD 内容是否合法；
    // 真正的打开与格式校验由 readPcdXYZ() 负责。
    if (std::filesystem::exists(candidate)) {
      return candidate;
    }
  }
  // 所有命名方式都未匹配时返回空 optional，调用方可以跳过缺失帧。
  return std::nullopt;
}

}  // namespace nav2_route3d
