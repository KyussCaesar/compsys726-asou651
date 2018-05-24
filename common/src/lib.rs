//! This crate simply has some common utilities and things for the COMPSYS 726
//! assignment.
//!
//! Author: Antony Southworth.

#[macro_use] pub extern crate rosrust;
#[macro_use] extern crate rosrust_codegen;

extern crate fnv;
extern crate rayon;

rosmsg_include!();

/// The prelude, which provides some common stuff.
pub mod prelude
{
    pub use rosrust;
    pub use msg;

    pub use rayon::prelude::*;

    /// The numeric type to use for all calculations.
    /// This should be either `f32` or `f64`.
    pub type Num = f64;

    pub use super::*;
}

/// Module containing utils for working with the OccupancyGrid.
pub mod map_utils
{
    use ::prelude::*;
    use ::std;

    /// An alias for the `OccupancyGrid` message type.
    pub type Map = msg::nav_msgs::OccupancyGrid;

    // The hashing algorithm to use.
    type Hasher = fnv::FnvBuildHasher;

    /// An alias for `HashMap`, using the fast FNV hashing algorithm.
    pub type HashMap<K, V> = std::collections::HashMap<K, V, Hasher>;

    /// An alias for `HashSet`, using the fast FNV hashing algorithm.
    pub type HashSet<V> = std::collections::HashSet<V, Hasher>;

    /// A pair of indices into the map.
    pub type Point = (usize, usize);

    /// A set of points.
    pub type Points = HashSet<Point>;

    /// Filters the map using the predicate.
    ///
    /// Returns a set of `(usize, usize)`; the row-column indices of the points
    /// which satisfy the predicate, i.e, for which the predicate is true.
    ///
    /// This function is handy because the map comes in as a 1D array, but the
    /// output of this function lets you think in terms of cell indices.
    pub fn filter_map<F>(map: &Map, f: F) -> Points
    where
        F: Fn(i8) -> bool + Sync
    {
        let len = map.info.height * map.info.width;

        (0..len).into_par_iter()
        .zip(map.data.par_iter())
        .filter_map(|(index, cell_value)|
        {
            if f(*cell_value)
            {
                let row = index / map.info.height;
                let col = index % map.info.width;

                Some((row as usize, col as usize))
            }

            else { None }
        })
        .collect()
    }

    // helper for transforming cell indices into map coordinates.
    fn tf_helper(map: &Map, p: Point) -> (Num, Num)
    {
        let row = p.0 as Num;
        let col = p.1 as Num;

        let height = map.info.height as Num;
        let width  = map.info.width  as Num;

        let res = map.info.resolution as Num;

        (
            -( ((width /2.0) - col) * res ),
             ( ((height/2.0) - row) * res ),
        )
    }

    /// Transforms cell indices into map coordinates.
    pub fn transform<Items: IntoIterator<Item=Point>>(map: &Map, items: Items) -> Vec<(Num, Num)>
    {
        items.into_iter().map(|p| tf_helper(map, p)).collect()
    }

    /// Transforms cell indices into map coordinates, in parallel.
    pub fn par_transform<Items: IntoParallelIterator<Item=Point>>(map: &Map, items: Items) -> Vec<(Num, Num)>
    {
        items.into_par_iter().map(|p| tf_helper(map, p)).collect()
    }
}
