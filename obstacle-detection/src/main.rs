//! # Obstacle Detection
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
    extract_groups,
};

/// The main callback that is passed to the subscriber object.
fn callback(map: Map)
{
    println!("recieved map, info: {:.4?}", map.info);

    let group_table = extract_groups(&map, |value| value > 3, 3);

    // we can now iterate over the groups of cells and try to determine whether
    // each group makes up a circle or a rectangle.
    for (_group, items) in group_table.into_iter()
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
