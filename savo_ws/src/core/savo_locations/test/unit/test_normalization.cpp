#include <gtest/gtest.h>

#include "savo_locations/normalization.hpp"


TEST(LocationNormalization, TrimsAsciiWhitespace)
{
  EXPECT_EQ(
    savo_locations::trim_ascii(
      " \t Room A201 \n"),
    "Room A201");
}


TEST(LocationNormalization, CollapsesWhitespace)
{
  EXPECT_EQ(
    savo_locations::
      collapse_ascii_whitespace(
        "  Room \t A201 \n East  "),
    "Room A201 East");
}


TEST(LocationNormalization, BuildsStableLookupKeys)
{
  EXPECT_EQ(
    savo_locations::normalize_lookup_key(
      "  Room-A_201 / East  "),
    "room a 201 east");

  EXPECT_EQ(
    savo_locations::normalize_lookup_key(
      "A201"),
    "a201");
}


TEST(LocationNormalization, CanonicalizesLocationIds)
{
  EXPECT_EQ(
    savo_locations::
      canonicalize_location_id(
        "  a 201  "),
    "A_201");

  EXPECT_EQ(
    savo_locations::
      canonicalize_location_id(
        "lab-west"),
    "LAB-WEST");
}


TEST(LocationNormalization, ValidatesCanonicalIds)
{
  EXPECT_TRUE(
    savo_locations::
      is_canonical_location_id(
        "A201"));

  EXPECT_TRUE(
    savo_locations::
      is_canonical_location_id(
        "LAB-WEST_2"));

  EXPECT_FALSE(
    savo_locations::
      is_canonical_location_id(
        "a201"));

  EXPECT_FALSE(
    savo_locations::
      is_canonical_location_id(
        "_A201"));

  EXPECT_FALSE(
    savo_locations::
      is_canonical_location_id(
        "A 201"));
}


TEST(LocationNormalization, PreservesUtf8Bytes)
{
  EXPECT_EQ(
    savo_locations::normalize_lookup_key(
      "  Käytävä 2  "),
    "käytävä 2");
}
