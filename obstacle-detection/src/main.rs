//! # Task 1 Part 1:
//! ## Obstacle Detection
//!
//! This module contains the definition of a ROS node for detecting objects
//! in the arena.
//!
//! It uses the `gmapping` node to build a map of the arena using a laser scanner,
//! and then processes the map in order to find the obstacles.

extern crate common;
use common::prelude::*;

mod model3;

use map_utils::
{
    Map,
    Point,
    Points,
    HashMap,

    filter_map,
};

use common::prelude::*;

/// Alias for `usize`.
type GroupNumber = usize;

/// A hash table that keeps track of contiguous blocks of occupied cells.
type GroupTable = HashMap<GroupNumber, Points>;

/// The main callback that is passed to the subscriber object.
fn callback(map: Map)
{
    println!("recieved map, info: {:.4?}", map.info);

    // we want to find contiguous blocks of "filled" cells. This tells us the 
    // edges of the shapes that we want to find.
    //
    // recursive flood-fill takes up too much memory and blows the stack real
    // quick, so instead we use a "towers-of-babylon" type approach, where
    // cell indices that need to be checked are moved from an index into a
    // "staging" area.
    //
    // For each cell in the set of occupied cells:
    // 
    // * move the cell from the occupied cells set into the "staging" set.
    // * that cell is added to the current group.
    // * find all the neighbours of that cell that are also occupied.
    // * those cells should also belong to the current group.
    // * finally, add the current cell to the "group" index.
    //
    // Once all the cells in the occupied set have been processed, we will now
    // have sets of points that are known to be close together and make up the 
    // edge of a shape. Now all we have to do is find out whether a given set 
    // of points makes up a rectangle or a circle.

    let mut current_group = 0;
    let mut staging = Vec::new();
    let mut group_table = GroupTable::default();

    let mut occupied_cells = filter_map(&map, |value| value > 3);

    while occupied_cells.len() != 0
    {
        // take the next one. Unfortunately, it seems there is no method for
        // this for HashSet.
        let mut iterator = occupied_cells.into_iter();
        let index = iterator.next().unwrap();
        occupied_cells = iterator.collect();

        staging.push(index);
        while let Some(current_index) = staging.pop()
        {
            // move all of the neighbours
            process_neighbours(current_index, &mut staging, &mut occupied_cells);
            group_table.entry(current_group).or_insert(Points::default()).insert(current_index);
        }

        current_group += 1;
    }

    // we can now iterate over the groups of cells and try to determine whether
    // each group makes up a circle or a rectangle.
    for (group, items) in group_table.into_iter()
    {
        if items.len() == 0
        {
            println!("Skipped a group that contained zero elements! (This should never happen).");
            continue;
        }

        // transform the items into xy, relative to the robot
        // starting position.
        let items = map_utils::par_transform(&map, items);

        // find the bounds of the box:
        let upper = items.par_iter().max_by(|a,b| a.0.partial_cmp(&b.0).unwrap()).unwrap();
        let lower = items.par_iter().min_by(|a,b| a.0.partial_cmp(&b.0).unwrap()).unwrap();
        let left  = items.par_iter().max_by(|a,b| a.1.partial_cmp(&b.1).unwrap()).unwrap();
        let right = items.par_iter().min_by(|a,b| a.1.partial_cmp(&b.1).unwrap()).unwrap();

        let box_size =
        {
            let bh = upper.0 - lower.0;
            let bw  = left.0 - right.0;

            bh.hypot(bw)
        };

        let a0 = left.0  as Num - lower.0 as Num;
        let a1 = left.1  as Num - lower.1 as Num;
        let b0 = right.0 as Num - lower.0 as Num;
        let b1 = right.1 as Num - lower.1 as Num;

        let a = a0.hypot(a1);
        let b = b0.hypot(b1);

        if a < 0.09 || b < 0.09 || box_size > 1.5
        {
            // assuming it's noise and quietly continuing.
            // this also filters for the borders
            continue;
        }

        println!("a0: {}", a0);
        println!("a1: {}", a1);
        println!("b0: {}", b0);
        println!("b1: {}", b1);
        println!("a:  {}", a);
        println!("b:  {}", b);

        println!("Bounding box:\nUpper: {:3.4}\t{:3.4}\nLower: {:3.4}\t{:3.4}\nLeft : {:3.4}\t{:3.4}\nRight: {:3.4}\t{:3.4}",
            upper.0, upper.1,
            lower.0, lower.1,
             left.0,  left.1,
            right.0, right.1);

        let shape = model3::hough_transform(
            &items,
            (lower.0 + (a0+b0)/2.0, lower.1 + (a1+b1)/2.0),
            a,
            b,
        );

        println!("{:?}", shape);
    }

    println!("Done processing map");
}

/// Helper for processing the neighbours of a cell.
fn process_neighbours(
    p: Point,
    staging: &mut Vec<Point>,
    occupied_cells: &mut Points
)
{
    let mut to_check: Points = Points::default();

    for i in 0..3
    {
        for j in 0..3
        {
            to_check.insert((p.0.saturating_add(i), p.1.saturating_add(j)));
            to_check.insert((p.0.saturating_add(i), p.1.saturating_sub(j)));
            to_check.insert((p.0.saturating_sub(i), p.1.saturating_add(j)));
            to_check.insert((p.0.saturating_sub(i), p.1.saturating_sub(j)));
        }
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
            println!("ERROR! Could not subscribe to /map: {:?}. Node is shutting down", e);
            return;
        }
    };

    println!("od2rs node successfully initialised");
    rosrust::spin();

    println!("od2rs shutting down");
}
