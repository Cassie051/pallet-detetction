#pragma once

#include <spds_geometry/polygon.h>
#include <spds_generic/serialization.h>

namespace boost {
  namespace serialization {
    class access;
    // --------------------------------- Polygon ----------------------------
    template<typename Archive>
      void serialize(Archive& ar, spds_geometry::Polygon& p, const unsigned version) {
      ar & p.points;
    }
    /* template<typename Archive> */
    /*   void save(Archive& ar, const spds_geometry::Polygon& obj, const unsigned version) { */
    /*   ar << obj.points; */
    /* } */
    
    /* template<typename Archive> */
    /*   void load(Archive& ar, spds_geometry::Polygon& obj, const unsigned version) { */
    /*   ar >> obj.points; */
    /* } */
  }
}


