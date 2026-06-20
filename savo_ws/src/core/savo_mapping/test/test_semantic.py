#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Unit tests for Robot Savo semantic mapping helpers."""

from __future__ import annotations

import json
from pathlib import Path
from tempfile import TemporaryDirectory

from savo_mapping.semantic import (
    CONFIRM_SOURCE_CLI,
    AprilTagObservationBatch,
    LocationPose,
    SemanticLandmarkStore,
    TagDatabase,
    apply_confirmation_to_store,
    apply_session_to_store,
    bridge_locations_from_known_locations,
    bridge_store_to_locations,
    find_bridge_location,
    filter_confirmed_candidates,
    filter_pending_candidates,
    latest_observations_by_tag_id,
    load_apriltag_mapper,
    load_apriltag_observation_batch,
    load_known_locations,
    load_location_record_set,
    load_semantic_landmark_store,
    load_tag_database,
    location_record_set_from_landmarks,
    make_apriltag_location_candidate,
    make_apriltag_observation,
    make_confirm_location,
    make_location_key,
    make_manual_location_session,
    make_rename_location,
    make_tag_record,
    save_apriltag_mapper,
    save_apriltag_observation_batch,
    save_known_locations,
    save_known_locations_from_records,
    save_location_record_set,
    save_semantic_landmark_store,
    save_tag_database,
)


def _candidate():
    return make_apriltag_location_candidate(
        tag_id=21,
        label="A201",
        x=4.2,
        y=8.7,
        confidence=0.95,
    )


def _confirmed_store() -> SemanticLandmarkStore:
    store = SemanticLandmarkStore(map_name="savonia_campus_heart")
    store = store.upsert_candidate(_candidate())
    return store.confirm_candidate("a201", confirmed_by="operator")


def test_make_location_key_normalizes_labels() -> None:
    assert make_location_key("A201") == "a201"
    assert make_location_key("A201 Entrance") == "a201_entrance"
    assert make_location_key("Info Desk!") == "info_desk"


def test_location_candidate_confirm_flow() -> None:
    candidate = _candidate()
    confirmed = candidate.confirm(confirmed_by="operator")

    items = (candidate,)
    items = (confirmed,)

    assert candidate.pending is True
    assert confirmed.confirmed is True
    assert confirmed.source == "apriltag"
    assert confirmed.tag_id == 21
    assert len(filter_pending_candidates(items)) == 0
    assert len(filter_confirmed_candidates(items)) == 1


def test_apriltag_observation_batch_roundtrip() -> None:
    obs1 = make_apriltag_observation(
        tag_id=21,
        x=4.2,
        y=8.7,
        confidence=0.95,
    )
    obs2 = make_apriltag_observation(
        tag_id=7,
        x=2.0,
        y=1.5,
        confidence=0.80,
    )

    batch = AprilTagObservationBatch(observations=(obs1, obs2))
    latest = latest_observations_by_tag_id(batch.observations)

    assert batch.count == 2
    assert batch.empty is False
    assert sorted(latest.keys()) == [7, 21]
    assert obs1.fresh(max_age_s=10.0) is True

    data = json.loads(batch.to_json())
    assert data["count"] == 2


def test_apriltag_observation_save_load() -> None:
    observation = make_apriltag_observation(
        tag_id=21,
        x=4.2,
        y=8.7,
        confidence=0.95,
    )
    batch = AprilTagObservationBatch(observations=(observation,))

    with TemporaryDirectory() as tmp:
        path = Path(tmp) / "apriltag_observations.json"
        save_apriltag_observation_batch(path, batch)
        loaded = load_apriltag_observation_batch(path)

    assert loaded.count == 1
    assert loaded.observations[0].tag_id == 21


def test_tag_database_creates_mapper() -> None:
    database = TagDatabase(map_name="savonia_campus_heart")
    database = database.upsert(
        make_tag_record(
            tag_id=21,
            label="A201",
            area_type="room_entrance",
            priority=3,
            aliases=("A201 Entrance", "Room A201"),
        )
    )

    mapper = database.to_apriltag_mapper()
    observation = make_apriltag_observation(
        tag_id=21,
        x=4.2,
        y=8.7,
        confidence=0.95,
    )
    result = mapper.map_observation(observation)

    assert database.count == 1
    assert database.find_by_key_or_label("Room A201").label == "A201"
    assert result.ok is True
    assert result.candidate is not None
    assert result.candidate.label == "A201"


def test_tag_database_save_load() -> None:
    database = TagDatabase(map_name="savonia_campus_heart")
    database = database.upsert(make_tag_record(tag_id=7, label="Info Desk"))

    with TemporaryDirectory() as tmp:
        path = Path(tmp) / "tag_database.json"
        save_tag_database(path, database)
        loaded = load_tag_database(path)

    assert loaded.count == 1
    assert loaded.find_by_id(7).label == "Info Desk"


def test_apriltag_mapper_save_load_from_tag_database() -> None:
    database = TagDatabase(map_name="savonia_campus_heart")
    database = database.upsert(make_tag_record(tag_id=21, label="A201"))

    mapper = database.to_apriltag_mapper()

    with TemporaryDirectory() as tmp:
        path = Path(tmp) / "apriltag_mapper.json"
        save_apriltag_mapper(path, mapper)
        loaded = load_apriltag_mapper(path)

    assert len(loaded.labels) == 1
    assert loaded.find_label(21).label == "A201"


def test_semantic_landmark_store_confirm_candidate() -> None:
    store = _confirmed_store()

    assert store.landmark_count == 1
    assert store.candidate_count == 1
    assert store.confirmed_candidate_count == 1
    assert store.find_landmark("a201").label == "A201"


def test_semantic_landmark_store_save_load() -> None:
    store = _confirmed_store()

    with TemporaryDirectory() as tmp:
        path = Path(tmp) / "semantic_landmarks.json"
        save_semantic_landmark_store(path, store)
        loaded = load_semantic_landmark_store(path)

    assert loaded.landmark_count == 1
    assert loaded.find_landmark("A201").label == "A201"


def test_human_label_session_rename_and_confirm() -> None:
    session = make_manual_location_session(
        label="A201",
        x=4.2,
        y=8.7,
        actor="operator",
    )

    session = session.rename("A201 Entrance")
    session = session.confirm(actor="operator")

    store = apply_session_to_store(SemanticLandmarkStore(), session)

    assert session.confirmed is True
    assert session.candidate.label == "A201 Entrance"
    assert session.candidate.key == "a201_entrance"
    assert store.landmark_count == 1
    assert store.find_landmark("a201_entrance").label == "A201 Entrance"


def test_location_confirmation_rename_and_confirm_store() -> None:
    store = SemanticLandmarkStore(map_name="savonia_campus_heart")
    store = store.upsert_candidate(_candidate())

    store, rename_result = apply_confirmation_to_store(
        store,
        make_rename_location(
            "a201",
            "A201 Entrance",
            source=CONFIRM_SOURCE_CLI,
        ),
    )

    store, confirm_result = apply_confirmation_to_store(
        store,
        make_confirm_location(
            "a201_entrance",
            source=CONFIRM_SOURCE_CLI,
        ),
    )

    assert rename_result.ok is True
    assert confirm_result.ok is True
    assert store.landmark_count == 1
    assert store.find_candidate("a201_entrance").label == "A201 Entrance"
    assert store.find_landmark("a201_entrance").label == "A201 Entrance"


def test_location_bridge_exports_known_locations() -> None:
    store = _confirmed_store()

    result = bridge_store_to_locations(
        store,
        aliases_by_key={"a201": ("A201 Entrance", "Room A201")},
    )
    known = result.to_known_locations_dict()
    locations = bridge_locations_from_known_locations(known)
    found = find_bridge_location(locations, "Room A201")

    assert result.ok is True
    assert result.location_count == 1
    assert list(known["locations"].keys()) == ["a201"]
    assert found is not None
    assert found.label == "A201"


def test_location_bridge_save_load_known_locations() -> None:
    store = _confirmed_store()
    known = bridge_store_to_locations(store).to_known_locations_dict()

    with TemporaryDirectory() as tmp:
        path = Path(tmp) / "known_locations.json"
        save_known_locations(path, known)
        loaded = load_known_locations(path)

    assert loaded["location_count"] == 1
    assert loaded["locations"]["a201"]["label"] == "A201"


def test_location_record_set_from_landmarks() -> None:
    store = _confirmed_store()

    records = location_record_set_from_landmarks(
        store.landmarks,
        map_name=store.map_name,
        aliases_by_key={"a201": ("A201 Entrance", "Room A201")},
        priority_by_key={"a201": 3},
    )

    found = records.find("Room A201")

    assert records.count == 1
    assert records.enabled_count == 1
    assert found is not None
    assert found.label == "A201"
    assert len(records.to_area_regions()) == 1
    assert len(records.to_semantic_targets()) == 1
    assert list(records.to_known_locations_dict()["locations"].keys()) == ["a201"]


def test_location_record_set_save_load_and_known_export() -> None:
    store = _confirmed_store()
    records = location_record_set_from_landmarks(
        store.landmarks,
        map_name=store.map_name,
    )

    with TemporaryDirectory() as tmp:
        record_path = Path(tmp) / "location_records.json"
        known_path = Path(tmp) / "known_locations.json"

        save_location_record_set(record_path, records)
        save_known_locations_from_records(known_path, records)

        loaded = load_location_record_set(record_path)

        assert known_path.exists()

    assert loaded.count == 1
    assert loaded.find("a201").label == "A201"


def test_location_pose_dict() -> None:
    pose = LocationPose(x=1.0, y=2.0, yaw=0.5, frame_id="map")

    assert pose.to_dict() == {
        "x": 1.0,
        "y": 2.0,
        "yaw": 0.5,
        "frame_id": "map",
    }
