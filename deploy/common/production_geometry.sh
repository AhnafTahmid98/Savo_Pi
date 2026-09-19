#!/usr/bin/env bash
# Shared fail-closed geometry policy for production Robot Savo runners.

savo_production_geometry_profile() {
  printf '%s\n' \
    "${SAVO_WS}/install/savo_description/share/savo_description/config/profiles/robot_savo_core_v1.yaml"
}

savo_assert_production_geometry_environment() {
  if [[ "${SAVO_REQUIRE_LOCKED_GEOMETRY:-true}" != true ]]; then
    savo_die \
      "Production geometry policy requires SAVO_REQUIRE_LOCKED_GEOMETRY=true"
  fi
  if [[ "${SAVO_ALLOW_PROVISIONAL_GEOMETRY:-false}" != false ]]; then
    savo_die \
      "Production geometry policy requires SAVO_ALLOW_PROVISIONAL_GEOMETRY=false"
  fi
}

savo_reject_production_geometry_arguments() {
  local argument
  for argument in "$@"; do
    case "${argument}" in
      geometry_profile:=*|require_locked_geometry:=*|allow_provisional_geometry:=*)
        savo_die "Production runner forbids overriding ${argument%%:=*}"
        ;;
    esac
  done
}
