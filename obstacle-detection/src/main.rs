//! # Task 1 Part 1:
//! ## Obstacle Detection
//!
//! This module contains the definition of a ROS node for detecting objects
//! in the arena.
//!
//! It uses the `gmapping` node to build a map of the arena using a laser scanner,
//! and then processes the map in order to find the obstacles.

#[macro_use] extern crate rosrust;
#[macro_use] extern crate rosrust_codegen;

extern crate fnv;
extern crate rayon;

use std::iter::Extend;

use rayon::prelude::*;

mod model;
use model::Model;

/// This gives access to ROS messages.
rosmsg_include!();

type Hasher = fnv::FnvBuildHasher;

/// An alias for `HashMap`, using the fast FNV hashing algorithm.
type HashMap<K, V> = std::collections::HashMap<K, V, Hasher>;

/// An alias for `HashSet`, using the fast FNV hashing algorithm.
type HashSet<V> = std::collections::HashSet<V, Hasher>;

/// A pair of indices into the map.
type Point = (usize, usize);

/// A set of points.
type Points = HashSet<Point>;

/// An alias for the `OccupancyGrid` message type.
type Map = msg::nav_msgs::OccupancyGrid;

/// Alias for `usize`.
type GroupNumber = usize;

/// A hash table that keeps track of contiguous blocks of occupied cells.
type GroupTable = HashMap<GroupNumber, Points>;

/// Iterates over the map and finds all of the occupied cells in the grid.
fn build_map_index(map: &Map) -> Points
{
    let len = map.info.height * map.info.width;

    // done in parallel to speed things up
    (0..len).into_par_iter()
    .zip(map.data.par_iter())
    .filter_map(|(index, cell_value)|
    {
        if *cell_value > 3
        {
            let i = index / map.info.height;
            let j = index % map.info.width;

            Some((j as usize, i as usize))
        }

        else { None }
    })
    .collect()
}


/// The main callback that is passed to the subscriber object.
fn callback(map: Map)
{
    println!("recieved map, info: {:?}", map.info);

    let mut current_group = 0;
    let mut staging = Vec::new();
    let mut group_table = GroupTable::default();

    let mut occupied_cells = build_map_index(&map);

    while occupied_cells.len() != 0
    {
        // take the next one.
        let mut iterator = occupied_cells.into_iter();
        let index = iterator.next().unwrap();
        occupied_cells = iterator.collect();

        staging.push(index);
        while let Some(current_index) = staging.pop()
        {
            process_neighbours(current_index, &mut staging, &mut occupied_cells);
            group_table.entry(current_group).or_insert(Points::default()).insert(current_index);
        }

        current_group += 1;
    }

    for (group, items) in group_table.iter()
    {
        if items.len() == 0
        {
            ros_info!("Skipped a group that contained zero elements! (This should never happen).");
            continue;
        }

        // find the bounds of the box:
        let lower = items.par_iter().min_by(|a,b| a.0.cmp(&b.0)).unwrap();
        let left  = items.par_iter().max_by(|a,b| a.1.cmp(&b.1)).unwrap();
        let right = items.par_iter().min_by(|a,b| a.1.cmp(&b.1)).unwrap();

        let a0 = left.0  as f32 - lower.0 as f32;
        let a1 = left.1  as f32 - lower.1 as f32;
        let b0 = right.0 as f32 - lower.0 as f32;
        let b1 = right.1 as f32 - lower.1 as f32;

        let a = (a0.powi(2) + a1.powi(2)).sqrt() * map.info.resolution;
        let b = (b0.powi(2) + b1.powi(2)).sqrt() * map.info.resolution;

        if a < 0.09 || b < 0.09 || a > 1.0 || b > 1.0
        {
            // assuming it's noise and quietly continuing.
            // this also filters for the borders
            continue;
        }

        let model = Model::fit(items, 0.02, a, b, b1.atan2(a1));

        ros_info!("Fitted a model!: {:?}", model);
    }

    ros_info!("Done processing map");
}

/// Helper for processing the neighbours of a cell.
fn process_neighbours(
    p: Point,
    staging: &mut Vec<Point>,
    occupied_cells: &mut Points
)
{
    // `(0 as usize) -1` is an easy way to cause a thread panic.
    // we do some gymnastics to check that we never do this.

    let mut to_check: Points = vec![
        (p.0 + 1, p.1    ),
        (p.0 + 2, p.1    ),
        (p.0    , p.1 + 1),
        (p.0    , p.1 + 2),
        (p.0 + 1, p.1 + 1),
        ].into_iter().collect();

    if p.0 > 0
    {
        to_check.extend(vec![(p.0 - 1, p.1), (p.0 - 1, p.1 + 1)]);
    }

    if p.0 > 1
    {
        to_check.extend(vec![(p.0 - 2, p.1)]);
    }

    if p.1 > 0
    {
        to_check.extend(vec![(p.0, p.1 - 1), (p.0 + 1, p.1 - 1)]);
    }

    if p.1 > 1
    {
        to_check.extend(vec![(p.0, p.1 - 2)]);
    }

    if p.0 > 0 && p.1 > 0
    {
        to_check.extend(vec![(p.0 - 1, p.1 - 1)]);
    }

    // add points that are "to_check" but still in occupied
    for point in occupied_cells.intersection(&to_check)
    {
        staging.push(*point);
    }

    // remove those cells from occupied list.
    to_check.iter().for_each(|p| { occupied_cells.remove(p); });
}

fn main()
{
    rosrust::init("od2rs");

    let _subscriber = match rosrust::subscribe("/map", callback)
    {
        Ok(s) => s,
        Err(e) =>
        {
            ros_info!("ERROR! Could not subscribe to /map: {:?}. Node is shutting down", e);
            return;
        }
    };

    println!("od2rs node successfully initialised");
    rosrust::spin();

    println!("od2rs shutting down");
}
