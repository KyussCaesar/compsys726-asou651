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

    /// Alias for `usize`.
    pub type GroupNumber = usize;

    /// A hash table that keeps track of contiguous blocks of cells which satisfied
    /// a predicate. See `extract_groups`.
    pub type GroupTable = HashMap<GroupNumber, Points>;

    /// Extracts groups from the map. A "group" is a contiguous block of cells
    /// whose value results in the predicate returning true.
    ///
    /// Recursive flood-fill takes up too much memory and blows the stack real
    /// quick, so instead we use a "towers-of-babylon" type approach, where
    /// cell indices that need to be checked are moved from an index into a
    /// "staging" area.
    ///
    /// For each cell in the set of cells which satisfy the predicate:
    /// 
    /// * move the cell from the cells set into the "staging" set.
    /// * that cell is added to the current group.
    /// * find all the neighbours of that cell that also satisfy the predicate.
    /// * those cells should also belong to the current group.
    /// * finally, add the current cell to the "group" index.
    ///
    /// Once all the cells in the occupied set have been processed, we will now
    /// have sets of points that are known to be close together and make up a 
    /// group.
    ///
    /// `kernel_size` is the region for which a cell is considered a "neighbour".
    pub fn extract_groups<F>(map: &Map, pred: F, kernel_size: usize) -> GroupTable
    where
        F: Fn(i8) -> bool + Sync
    {
        // first, get the whole set of cells which satisfy the predicate
        let mut cells = filter_map(map, pred);

        // now, initialise some stuff
        let mut current_group = 0;
        let mut staging = Vec::new();
        let mut group_table = GroupTable::default();

        // we play "towers of babylon".
        while cells.len() != 0
        {
            // take the next one. Unfortunately, it seems there is no method for
            // this for HashSet.
            let mut iterator = cells.into_iter();
            let index = iterator.next().unwrap();
            cells = iterator.collect();

            staging.push(index);
            while let Some(current_index) = staging.pop()
            {
                // move all of the neighbours
                process_neighbours(current_index, &mut staging, &mut cells, kernel_size);
                group_table.entry(current_group).or_insert(Points::default()).insert(current_index);
            }

            current_group += 1;
        }

        return group_table;
    }

    // Helper for extract_groups
    fn process_neighbours(
        p: Point,
        staging: &mut Vec<Point>,
        cells: &mut Points,
        kernel_size: usize,
    )
    {
        let mut to_check = neighbours(p, kernel_size);

        // add points that are "to_check" but still in occupied
        cells.intersection(&to_check).for_each(|p| staging.push(*p));

        // remove those cells from list.
        to_check.iter().for_each(|p| { cells.remove(p); });
    }

    /// Returns the set of neighbours of a cell.
    pub fn neighbours(
        p: Point,
        kernel_size: usize,
    ) -> Points
    {
        let mut neighbours: Points = Points::default();

        for i in 0..kernel_size
        {
            for j in 0..kernel_size
            {
                neighbours.insert((p.0.saturating_add(i), p.1.saturating_add(j)));
                neighbours.insert((p.0.saturating_add(i), p.1.saturating_sub(j)));
                neighbours.insert((p.0.saturating_sub(i), p.1.saturating_add(j)));
                neighbours.insert((p.0.saturating_sub(i), p.1.saturating_sub(j)));
            }
        }

        return neighbours;
    }
}
