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

/// This gives access to ROS messages.
rosmsg_include!();

type Hasher = fnv::FnvBuildHasher;

/// An alias for `HashMap`, using the fast FNV hashing algorithm.
type HashMap<K, V> = std::collections::HashMap<K, V, Hasher>;

/// An alias for `HashSet`, using the fast FNV hashing algorithm.
type HashSet<V> = std::collections::HashSet<V, Hasher>;

/// A pair of indices into the map.
type Pair = (usize, usize);

/// A set of points.
type Points = HashSet<Pair>;

/// An alias for the `OccupancyGrid` message type.
type Map = msg::nav_msgs::OccupancyGrid;

/// An alias for a type representing the cells that are occupied.
type OccupiedCells = HashSet<Pair>;

/// Alias for `usize`.
type GroupNumber = usize;

/// A hash table that keeps track of contiguous blocks of occupied cells.
type GroupTable = HashMap<GroupNumber, Vec<Pair>>;

/// Iterates over the map and finds all of the occupied cells in the grid.
fn build_map_index(map: &Map) -> OccupiedCells
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
            group_table.entry(current_group).or_insert(Vec::new()).push(current_index);
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
        let upper = items.par_iter().max_by(|a,b| a.0.cmp(&b.0)).unwrap();
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

        ros_info!("vec a: {:?} vec b: {:?}", (a0, a1), (b0, b1));
        ros_info!("Found bounding-box for Group {:?}: {:?} {:?} {:?} {:?}",
            group, upper, lower, left, right);

        // check angle between the primary vectors
        // if it's sufficiently away from 90 degrees then it's a circle.
        let dot_product = ((a0 * b0) + (a1 * b1)).abs() / a*b*map.info.resolution.powi(2);

        ros_info!("dot product: {:?}", dot_product);

        if dot_product < 0.03
        {
            ros_info!("Found rectangle!");

            ros_info!("Estimated length of side a: {:?} m", a);
            ros_info!("Estimated length of side b: {:?} m", b);

            let com0 = lower.0 as f32 + (a0 + b0) / 2.0;
            let com1 = lower.1 as f32 + (a1 + b1) / 2.0;
            let com  = ((com0 - (map.info.width  as f32)/2.0) * map.info.resolution,
                        (com1 - (map.info.height as f32)/2.0) * map.info.resolution);

            ros_info!("Estimated CoM in XY: {:?}", com);
        }

        else
        {
            ros_info!("Found circle!");
            
            // find the middle point
            let mid_index = (left.1 + right.1) / 2;
            let centre =
                items
                .par_iter()
                .filter(|index| mid_index < index.1)
                .min_by_key(|index| index.0)
                .unwrap();

            let p1 = left;
            let p2 = centre;
            let p3 = right;

            ros_info!("P1: {:?}", p1);
            ros_info!("P2: {:?}", p2);
            ros_info!("P3: {:?}", p3);

            let ma: f32 = (p2.1 - p1.1) as f32 / (p2.0 - p1.0) as f32;
            let mb: f32 = (p3.1 - p2.1) as f32 / (p3.0 - p2.0) as f32;

            let term1: f32 = (p1.1 - p3.1) as f32;
            let term2: f32 = (p1.0 + p2.0) as f32;
            let term3: f32 = (p2.0 + p3.0) as f32;

            // centre of the circle
            let x: f32 = (ma*mb*term1 + mb*term2 - ma*term3) / (2.0*(mb - ma));
            let y: f32 = (-1.0/ma) * (x - (p1.0 + p2.0) as f32 / 2.0) + (p1.1 + p2.1) as f32 / 2.0;

            let x_cm: f32 = (x - (map.info.width as f32) / 2.0) * map.info.resolution;
            let y_cm: f32 = (y - (map.info.height as f32) / 2.0) * map.info.resolution;

            ros_info!("Estimated CoM in XY: {:?}", (x_cm, y_cm));

            // calculate the radius by vector difference
            let tx = (p1.0 as f32 - x) as f32;
            let ty = (p1.1 as f32 - y) as f32;
            let radius = (tx.powi(2) + ty.powi(2)).sqrt() * map.info.resolution;

            ros_info!("Estimated radius: {:?}", radius);
        }
    }

    ros_info!("Done processing map");
}

/// Helper for processing the neighbours of a cell.
fn process_neighbours(
    p: Pair,
    staging: &mut Vec<Pair>,
    occupied_cells: &mut OccupiedCells
)
{
    // `(0 as usize) -1` is an easy way to cause a thread panic.
    // we do some gymnastics to check that we never do this.

    let mut to_check: HashSet<Pair> = vec![
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
