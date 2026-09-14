from route3d_topology_core import apply_topology_schema


def test_schema_defaults_are_added_without_losing_existing_fields():
    document = {
        "version": 7,
        "vertices": {
            "1": {
                "pos": [0.0, 0.0, 0.0],
                "meta": {"isCorner": True, "source": "test"},
            },
        },
        "edges": {
            "4": {
                "v": [1, 2],
                "meta": {"dir": 1, "source": "loop_closure"},
            },
        },
        "generation": {"loop_closure_count": 1},
    }

    result = apply_topology_schema(document)

    assert result is document
    assert result["schema"] == {"name": "route3d_topology", "version": 2}
    assert result["sceneMode"] == "normal"
    assert result["vertices"]["1"]["meta"]["isCorner"] is True
    assert result["vertices"]["1"]["meta"]["chargingMode"] == 0
    assert result["vertices"]["1"]["alignFinalYaw"] is False
    assert result["vertices"]["1"]["mustPassThrough"] is True
    assert result["vertices"]["1"]["passRadiusM"] == 0.28
    edge = result["edges"]["4"]
    assert edge["rotationAllowed"] is True
    assert edge["meta"]["travelMode"] == "first_to_second"
    assert edge["meta"]["obstacleMode"] == 0
    assert "locomotionMode" not in edge["meta"]
    assert edge["meta"]["linearSpeedMps"] == 1.0
    assert result["generation"]["loop_closure_count"] == 1


def test_schema_application_is_idempotent_and_preserves_overrides():
    document = {
        "sceneMode": "tunnel",
        "vertices": {
            "1": {
                "meta": {"chargingMode": 2},
                "alignFinalYaw": False,
                "mustPassThrough": True,
                "passRadiusM": 0.12,
            },
        },
        "edges": {
            "1": {
                "meta": {
                    "dir": 0,
                    "linearSpeedMps": 0.35,
                    "controllerMode": "pid",
                },
                "rotationAllowed": False,
            },
        },
    }

    apply_topology_schema(document)
    apply_topology_schema(document)

    assert document["sceneMode"] == "tunnel"
    assert document["vertices"]["1"]["meta"]["chargingMode"] == 2
    assert document["vertices"]["1"]["alignFinalYaw"] is False
    assert document["vertices"]["1"]["mustPassThrough"] is True
    assert document["vertices"]["1"]["passRadiusM"] == 0.12
    assert document["edges"]["1"]["rotationAllowed"] is False
    assert document["edges"]["1"]["meta"]["linearSpeedMps"] == 0.35
    assert document["edges"]["1"]["meta"]["controllerMode"] == "pid"
