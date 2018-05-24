#![allow(non_snake_case)]

use ::common::prelude::*;

type Point = (Num, Num);
type Points = Vec<Point>;
type Range  = Vec<Num>;

use std::f64::INFINITY;

#[derive(Debug)]
pub enum Shape
{
    Circle(Circle),
    Rectle(Rectle),
}


#[derive(Debug)]
pub struct Circle
{
    pub centre: Point,
    pub radius: Num,
    pub score:  Num,
}

impl Circle
{
    fn new() -> Self
    {
        Circle
        {
            centre: (0.0, 0.0),
            radius: 0.0,
            score:  INFINITY,
        }
    }
}


#[derive(Debug)]
pub struct Rectle
{
    pub centre: Point,
    pub width: Num,
    pub length: Num,
    pub rotation: Num,
    pub score: Num
}

impl Rectle
{
    fn new() -> Self
    {
        Rectle
        {
            centre: (0.0, 0.0),
            width: 0.0,
            length:  0.0,
            rotation: 0.0,
            score: INFINITY,
        }
    }

    fn from(points: &Points, a: Num, b: Num, p: Num, q: Num, t: Num) -> Self
    {
        Rectle
        {
            centre: (p, q),
            width: a,
            length: b,
            rotation: t,
            score: ht_score(points, a, b, p, q, t, 6),
        }
    }
}


/// Hough-transform inspired parameter search.
pub fn hough_transform(points: &Points, start: Point, a: Num, b: Num) -> Shape
{
    println!("HT starting from position: {:?}, a: {}, b: {}", start, a, b);

    // circles add the constraint that a == b, which restricts the size of the
    // parameter space. This makes the parameter search a lot easier, so we
    // do this one first.
    let circle = fit_circle(points, start, a+b / 2.0);

    // early return if it looks like a circle
    if circle.score < 0.002 { return Shape::Circle(circle) }

    // otherwise, check for rectangle
    let rectle = fit_rectle(points, start, a, b);

    // we want the min of the scores
    if rectle.score < circle.score
    {
        return Shape::Rectle(rectle);
    }

    return Shape::Circle(circle);
}

fn fit_rectle(points: &Points, start: Point, a: Num, b: Num) -> Rectle
{
    println!("fit rectle");

    let p = start.0;
    let q = start.1;

    let pq_width = 0.020;
    let ab_width = 0.020;

    let min: Rectle            = range(a - ab_width, a + ab_width, 0.010).into_par_iter()
    .flat_map(|aa              | range(b - ab_width, b + ab_width, 0.010).into_par_iter().map(|bb| (aa, bb)             ).collect::<Vec<_>>())
    .flat_map(|(aa, bb)        | range(p - pq_width, p + pq_width, 0.010).into_par_iter().map(|pp| (aa, bb, pp)         ).collect::<Vec<_>>())
    .flat_map(|(aa, bb, pp)    | range(q - pq_width, q + pq_width, 0.010).into_par_iter().map(|qq| (aa, bb, pp, qq)     ).collect::<Vec<_>>())
    .flat_map(|(aa, bb, pp, qq)| range(         0.0,        1.574, 0.010).into_par_iter().map(|tt| (aa, bb, pp, qq, tt) ).collect::<Vec<_>>())
    .map(|(a, b, p, q, t)| Rectle::from(points, a, b, p, q, t))
    .min_by(|a,b| a.score.partial_cmp(&b.score).unwrap()).unwrap();

    println!("min rectle: {:?} (rot: {})", min, min.rotation.to_degrees());

    min
}

fn fit_circle(points: &Points, start: Point, r: Num) -> Circle
{
    println!("fit circle");

    let mut min = Circle::new();

    for rr in range(r - 0.1, r + 0.1, 0.01)
    {
        for pp in range(start.0 - 0.3, start.0 + 0.3, 0.02)
        {
            for qq in range(start.1 - 0.3, start.1 + 0.3, 0.02)
            {
                let score = ht_score(points, rr, rr, pp, qq, 0.0, 1);

                if score < min.score
                {
                    min.centre = (pp, qq);
                    min.radius = rr;
                    min.score  = score;
                }
            }
        }
    }

    println!("min circle: {:?}", min);

    min
}

/// Evaluates the score of the model against the points, given the parameters.
/// Lower is better.
fn ht_score(points: &Points, a: Num, b: Num, p: Num, q: Num, t: Num, s: i32) -> Num
{
    let f = |x: Num| x - p;
    let g = |y: Num| y - q;

    let R = |x: Num, y: Num| f(x)*t.cos() + g(y)*t.sin();
    let C = |x: Num, y: Num| g(y)*t.cos() - f(x)*t.sin();

    let A = |x: Num, y: Num| R(x,y) / a;
    let B = |x: Num, y: Num| C(x,y) / b;

    let X = |x: Num, y: Num| A(x,y).powi(2*s);
    let Y = |x: Num, y: Num| B(x,y).powi(2*s);

    let M = |p: &(Num, Num)| (X(p.0, p.1) + Y(p.0, p.1) - 1.0).powi(2) / (X(p.0, p.1) + Y(p.0, p.1));

    let len = points.len() as Num;

    let T = |p: &(Num, Num)| (M(p) / s as Num).tanh() / len;

    return points.par_iter().map(T).sum();
}


fn range(start: Num, stop: Num, step: Num) -> Range
{
    let mut vec = Vec::new();
    let mut acc = start;

    while acc < stop
    {
        vec.push(acc);
        acc += step;
    }

    vec
}

