#!/usr/bin/env python3
"""Build an audited, binary-only ROS 2 Jazzy Debian package.

Run on the private build machine. Neither this script nor the source tree is
copied to the customer package. Python implementations are compiled with
Cython; only small generated ROS entry-point shims remain as Python text.
"""

from __future__ import annotations

import argparse
import ast
import os
from pathlib import Path
import re
import shutil
import stat
import subprocess
import sys
import tempfile


WORKSPACE = Path(__file__).resolve().parents[2]
PREFIX = Path('/opt/route3d')
MARKER = '# route3d-generated-wrapper\n'
PYTHON = Path('/usr/bin/python3')


def run(*args: str, **kwargs: object) -> None:
    subprocess.run([str(arg) for arg in args], check=True, **kwargs)


def build_colcon(work_dir: Path) -> Path:
    install = work_dir / 'install'
    env = os.environ.copy()
    path_flags = (f'-ffile-prefix-map={WORKSPACE}=. '
                  f'-ffile-prefix-map={work_dir}=. -g0')
    env['CFLAGS'] = f"{env.get('CFLAGS', '')} {path_flags}"
    env['CXXFLAGS'] = f"{env.get('CXXFLAGS', '')} {path_flags}"
    command = (
        'source /opt/ros/jazzy/setup.bash && '
        'colcon --log-base "$1" build --base-paths "$2" '
        '--build-base "$3" --install-base "$4" --merge-install '
        '--cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF'
    )
    run('bash', '-c', command, 'route3d-build', work_dir / 'log',
        WORKSPACE / 'src', work_dir / 'build', install, env=env)
    return install


def copy_runtime(install: Path, root: Path) -> Path:
    if not (install / 'setup.bash').is_file():
        raise RuntimeError(f'Not a completed colcon install: {install}')
    target = root / PREFIX.relative_to('/')
    target.mkdir(parents=True)
    shutil.copytree(install, target / 'install', symlinks=True)
    for cache in (target / 'install').rglob('__pycache__'):
        if cache.is_dir():
            shutil.rmtree(cache)
    for path in (target / 'install').rglob('*'):
        if path.is_file() and path.suffix in {'.pyc', '.pyo'}:
            path.unlink()
    old_prefix = str(install).encode()
    new_prefix = str(PREFIX / 'install').encode()
    for path in (target / 'install').rglob('*'):
        if not path.is_file() or path.is_symlink():
            continue
        content = path.read_bytes()
        if old_prefix not in content:
            continue
        if b'\0' in content:
            raise RuntimeError(f'Build prefix is embedded in binary: {path}')
        path.write_bytes(content.replace(old_prefix, new_prefix))
    for path in (target / 'install').rglob('*.rviz'):
        path.write_text(path.read_text().replace(
            str(WORKSPACE / 'data'), '/var/lib/route3d'))
    scripts = target / 'sh'
    scripts.mkdir()
    for source in (WORKSPACE / 'sh').glob('[0-9]*_*.sh'):
        shutil.copy2(source, scripts / source.name)
        run('bash', '-n', scripts / source.name)
    for source in (WORKSPACE / 'sh').glob('*.yaml'):
        shutil.copy2(source, scripts / source.name)
    (root / 'var/lib/route3d').mkdir(parents=True)
    (target / 'data').symlink_to('/var/lib/route3d', target_is_directory=True)
    return target


def prepare_private_python(install: Path) -> list[tuple[str, Path]]:
    """Replace installed Python implementations with native extensions."""
    site = install / 'lib/python3.12/site-packages'
    if not site.is_dir():
        raise RuntimeError(f'Python 3.12 site-packages missing: {site}')
    extensions: list[tuple[str, Path]] = []

    launch_private = site / 'route3d_private_launches'
    scripts_private = site / 'route3d_private_scripts'
    launch_private.mkdir()
    scripts_private.mkdir()
    (launch_private / '__init__.py').write_text(MARKER)
    (scripts_private / '__init__.py').write_text(MARKER)

    for path in sorted(install.glob('share/*/launch/*.launch.py')):
        package = path.parts[-3]
        name = re.sub(r'\W+', '_', path.name.removesuffix('.launch.py'))
        module = f'{package}_{name}'
        private_source = launch_private / f'{module}.py'
        shutil.copy2(path, private_source)
        path.write_text(MARKER +
                        f'from route3d_private_launches.{module} import generate_launch_description\n')
        extensions.append((f'route3d_private_launches.{module}', private_source))

    for path in sorted(install.glob('lib/*/*')):
        if not path.is_file() or path.is_symlink():
            continue
        with path.open('rb') as stream:
            head = stream.read(256)
        if not head.startswith((b'#!/usr/bin/python', b'#!/usr/bin/env python')):
            continue
        if b'EASY-INSTALL-ENTRY-SCRIPT' in head:
            continue  # setuptools entry point; implementation is compiled below
        content = path.read_text()
        tree = ast.parse(content, filename=str(path))
        if not any(isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                   and node.name == 'main' for node in tree.body):
            # Several ROS packages install tiny handwritten entry points that
            # only import main() from the package compiled below.
            imports_main = any(
                isinstance(node, ast.ImportFrom)
                and node.module and node.module.startswith(path.parent.name + '.')
                and any(alias.name == 'main' for alias in node.names)
                for node in tree.body)
            if len(content) > 512 or not imports_main:
                raise RuntimeError(f'Python executable has no main(): {path}')
            path.write_text('#!/usr/bin/python3\n' + MARKER +
                            content.split('\n', 1)[1])
            continue
        package = path.parent.name
        name = re.sub(r'\W+', '_', path.name)
        module = f'{package}_{name}'
        private_source = scripts_private / f'{module}.py'
        private_source.write_text(content)
        path.write_text('#!/usr/bin/python3\n' + MARKER +
                        f'from route3d_private_scripts.{module} import main\n'
                        'raise SystemExit(main())\n')
        path.chmod(path.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
        extensions.append((f'route3d_private_scripts.{module}', private_source))

    for path in sorted(site.rglob('*.py')):
        if launch_private in path.parents or scripts_private in path.parents:
            continue
        relative = path.relative_to(site).with_suffix('')
        if path.name == '__init__.py':
            module = '.'.join((*relative.parts[:-1], '_route3d_init'))
        else:
            module = '.'.join(relative.parts)
        extensions.append((module, path))
    return extensions


def compile_python(install: Path, work_dir: Path) -> None:
    try:
        from Cython.Build import cythonize
        from setuptools import Extension, setup
    except ImportError as error:
        raise RuntimeError('Build machine requires Python 3 Cython and setuptools') from error

    site = install / 'lib/python3.12/site-packages'
    planned = prepare_private_python(install)
    originals = [(module, path, path.name == '__init__.py') for module, path in planned]
    generated_source_root = f'{work_dir}/cython-c{work_dir}'
    native = [Extension(module, [str(path)], extra_compile_args=[
        '-O2', '-g0',
        f'-ffile-prefix-map={work_dir}=.',
        f'-ffile-prefix-map={generated_source_root}=.'])
              for module, path, _ in originals]
    print(f'Compiling {len(native)} Python modules with Cython', flush=True)
    compiled = cythonize(native, nthreads=min(os.cpu_count() or 2, 4),
                         quiet=True, build_dir=str(work_dir / 'cython-c'),
                         compiler_directives={'language_level': 3,
                                              'binding': False,
                                              'embedsignature': False,
                                              'emit_code_comments': False})
    # Cython embeds its input filename in tracebacks. Use the installed path,
    # never the private build machine's staging path.
    for extension in compiled:
        for filename in extension.sources:
            generated = Path(filename)
            if generated.suffix in {'.c', '.cc', '.cpp'}:
                content = generated.read_text()
                generated.write_text(content.replace(
                    str(install), str(PREFIX / 'install')))
    setup(name='route3d-private-runtime', ext_modules=compiled,
          script_args=['-q', 'build_ext', '-j', '4', '--build-lib', str(site),
                       '--build-temp', str(work_dir / 'cython-obj')])
    for _, path, is_init in originals:
        if is_init:
            path.write_text(MARKER + 'from ._route3d_init import *\n')
        else:
            path.unlink()


def remove_development_files(install: Path) -> None:
    for directory in install.glob('lib/python*/site-packages/test'):
        shutil.rmtree(directory)
    for directory in install.glob('share/*/cmake'):
        shutil.rmtree(directory)
    for directory in install.glob('share/*/docs'):
        shutil.rmtree(directory)
    for directory in install.glob('share/*/examples'):
        shutil.rmtree(directory)
    if (install / 'include').exists():
        shutil.rmtree(install / 'include')
    for path in list(install.rglob('*')):
        if path.name == '__pycache__' and path.is_dir():
            shutil.rmtree(path)
        elif path.is_file() and path.suffix in {
                '.a', '.c', '.cc', '.cpp', '.h', '.hpp', '.o', '.pyc', '.pyo', '.pxd', '.pyx'}:
            path.unlink()
    for path in install.glob('share/*/README*'):
        path.unlink()
    for path in install.rglob('*.egg-info/SOURCES.txt'):
        path.unlink()


def move_editable_yaml(root: Path, target: Path) -> list[str]:
    install = target / 'install'
    candidates = list(install.glob('share/*/config/**/*.yaml'))
    candidates += list(install.glob('share/*/config/**/*.yml'))
    candidates += list((target / 'sh').glob('*.yaml'))
    conffiles: list[str] = []
    for path in sorted(candidates):
        if path.is_symlink():
            raise RuntimeError(f'Config is already a symlink: {path}')
        if path.parent == target / 'sh':
            name = Path('sh') / path.name
        else:
            relative = path.relative_to(install / 'share')
            name = Path(relative.parts[0]) / Path(*relative.parts[2:])
        absolute = Path('/etc/route3d') / name
        destination = root / absolute.relative_to('/')
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.move(path, destination)
        path.symlink_to(absolute)
        conffiles.append(str(absolute))

    web = root / 'etc/route3d/route3d_web_console/web_console.yaml'
    if web.is_file():
        source_root = '/home/langyi/workspace/wyf/topo_graph_m20_ws'
        replacements = {
            f'{source_root}/src/route3d_product_demo/config/live_route_product.yaml':
                '/etc/route3d/route3d_product_demo/live_route_product.yaml',
            f'{source_root}/src/route3d_odom_waypoint/config/odom_waypoint.yaml':
                '/etc/route3d/route3d_odom_waypoint/odom_waypoint.yaml',
            f'{source_root}/install/setup.bash': '/opt/route3d/install/setup.bash',
            f'{source_root}/sh/': '/opt/route3d/sh/',
            f'{source_root}/data': '/var/lib/route3d',
        }
        content = web.read_text()
        for before, after in replacements.items():
            content = content.replace(before, after)
        if f'{source_root}/src/' in content:
            raise RuntimeError('Web config still refers to source tree')
        web.write_text(content)
    source_root = '/home/langyi/workspace/wyf/topo_graph_m20_ws'
    catalog = target / 'install/share/route3d_web_console/config/map_catalog.json'
    if catalog.is_file():
        catalog.write_text(catalog.read_text().replace(
            f'{source_root}/data', '/var/lib/route3d'))
    for script in (target / 'sh').glob('*.sh'):
        script.write_text(script.read_text().replace(
            f'{source_root}/data', '/var/lib/route3d'))
    return conffiles


def audit(root: Path) -> None:
    errors: list[str] = []
    staging_prefix = str(root.parent).encode() if root.name == 'debroot' else b''
    for path in root.rglob('*'):
        relative = path.relative_to(root)
        if path.is_symlink():
            target = os.readlink(path)
            if not target.startswith('/etc/route3d/') and target != '/var/lib/route3d':
                errors.append(f'unexpected symlink: {relative} -> {target}')
            continue
        if not path.is_file():
            continue
        if path.suffix in {'.cpp', '.cc', '.c', '.h', '.hpp', '.a', '.o', '.pyc', '.pyo',
                           '.pyx', '.pxd', '.ts', '.tsx', '.jsx', '.vue', '.map'}:
            errors.append(f'source/build file: {relative}')
        if path.suffix == '.py':
            content = path.read_text(errors='replace')
            generated = MARKER in content and len(content) < 1024
            entry_point = 'EASY-INSTALL-ENTRY-SCRIPT' in content and len(content) < 4096
            colcon_helper = path.name in {'_local_setup_util_sh.py', '_local_setup_util_ps1.py'}
            if not (generated or entry_point or colcon_helper):
                errors.append(f'uncompiled Python: {relative}')
        if path.suffix in {'.yaml', '.yml', '.sh', '.py', '.xml', '.dsv', '.cfg'}:
            content = path.read_text(errors='replace')
            if str(WORKSPACE) in content:
                errors.append(f'build machine path: {relative}')
        payload = path.read_bytes()
        if str(WORKSPACE).encode() in payload:
            errors.append(f'build machine path in file: {relative}')
        if staging_prefix and staging_prefix in payload:
            errors.append(f'staging path in file: {relative}')
    if errors:
        raise RuntimeError('Release audit failed:\n' + '\n'.join(errors[:80]))


def strip_elf(install: Path) -> None:
    for path in install.rglob('*'):
        if not path.is_file() or path.is_symlink():
            continue
        with path.open('rb') as stream:
            if stream.read(4) != b'\x7fELF':
                continue
        run('strip', '--strip-unneeded', path)


def package_deb(root: Path, output: Path, version: str, conffiles: list[str]) -> None:
    metadata = root / 'DEBIAN'
    metadata.mkdir()
    (metadata / 'control').write_text(
        'Package: route3d-m20-runtime\n'
        f'Version: {version}\n'
        'Section: misc\nPriority: optional\nArchitecture: amd64\n'
        'Maintainer: Route3D <release@example.invalid>\n'
        'Depends: python3 (>= 3.12), ros-jazzy-ros-base\n'
        'Description: Route3D M20 binary runtime for ROS 2 Jazzy\n'
    )
    (metadata / 'conffiles').write_text(''.join(f'{path}\n' for path in conffiles))
    postinst = metadata / 'postinst'
    postinst.write_text('#!/bin/sh\nset -e\n'
                        'if id langyi >/dev/null 2>&1; then\n'
                        '  chown langyi:langyi /var/lib/route3d\n'
                        'fi\n')
    postinst.chmod(0o755)
    output.parent.mkdir(parents=True, exist_ok=True)
    run('dpkg-deb', '--build', '--root-owner-group', root, output)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--version', default='0.1.0-1')
    parser.add_argument('--output', type=Path, default=WORKSPACE / 'dist/route3d-m20-runtime.deb')
    parser.add_argument('--work-dir', type=Path)
    parser.add_argument('--reuse-install', type=Path,
                        help='Use an existing non-symlink colcon install for a packaging iteration')
    args = parser.parse_args()
    if args.work_dir:
        work = args.work_dir.resolve()
        work.mkdir(parents=True, exist_ok=True)
        build(work, args)
    else:
        with tempfile.TemporaryDirectory(prefix='route3d-release-') as directory:
            build(Path(directory), args)


def build(work: Path, args: argparse.Namespace) -> None:
    install = args.reuse_install.resolve() if args.reuse_install else build_colcon(work)
    for directory in (work / 'cython-c', work / 'cython-obj'):
        if directory.exists():
            shutil.rmtree(directory)
    root = work / 'debroot'
    if root.exists():
        shutil.rmtree(root)
    root.mkdir()
    target = copy_runtime(install, root)
    runtime_install = target / 'install'
    compile_python(runtime_install, work)
    remove_development_files(runtime_install)
    conffiles = move_editable_yaml(root, target)
    strip_elf(runtime_install)
    audit(root)
    package_deb(root, args.output.resolve(), args.version, conffiles)
    print(f'Built and audited {args.output.resolve()}')


if __name__ == '__main__':
    main()
