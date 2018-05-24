#![allow(non_snake_case)]
#![allow(dead_code)]

use rayon::prelude::*;

pub type Num = f64;

type Point  = (Num, Num);
type Points = Vec<Point>;


#[derive(Debug)]
pub struct Model
{
    pub a: Num,
    pub b: Num,
    pub p: Num,
    pub q: Num,
    pub s: Num,
    pub t: Num,
}

impl Model
{
    pub fn M(&self, x: Num, y: Num) -> Num
    {
        self.X(x, y) + self.Y(x, y) - 1.0
    }

    fn X(&self, x: Num, y: Num) -> Num
    {
        self.A(x, y).powi((2.0*self.s).round() as i32)
    }

    fn Y(&self, x: Num, y: Num) -> Num
    {
        self.B(x, y).powi((2.0*self.s).round() as i32)
    }

    fn A(&self, x: Num, y: Num) -> Num
    {
        self.R(x, y) / self.a
    }

    fn B(&self, x: Num, y: Num) -> Num
    {
        self.C(x, y) / self.b
    }

    fn R(&self, x: Num, y: Num) -> Num
    {
        self.F(x) * self.t.cos() + self.G(y) * self.t.sin()
    }

    fn C(&self, x: Num, y: Num) -> Num
    {
        self.G(y) * self.t.cos() - self.F(x) * self.t.sin()
    }

    fn F(&self, x: Num) -> Num
    {
        x - self.p
    }

    fn G(&self, y: Num) -> Num
    {
        y - self.q
    }

    pub fn dMda(&self, x: Num, y: Num) -> Num
    {
        self.dXda(x,y) + self.dYda()
    }

    fn dXda(&self, x: Num, y: Num) -> Num
    {
        self.dXdA(x,y) * self.dAda(x,y)
    }

    fn dXdA(&self, x: Num, y: Num) -> Num
    {
        2.0 * self.s * self.A(x,y).powi((2.0 * self.s) as i32 - 1)
    }

    fn dAda(&self, x: Num, y: Num) -> Num
    {
        -self.R(x,y) / self.a.powi(2)
    }

    fn dYda(&self) -> Num
    {
        0.0
    }

    pub fn dMdb(&self, x: Num, y: Num) -> Num
    {
        self.dXdb() + self.dYdb(x,y)
    }

    fn dXdb(&self) -> Num
    {
        0.0
    }

    fn dYdb(&self, x: Num, y: Num) -> Num
    {
        self.dYdB(x,y) * self.dBdb(x,y)
    }

    fn dYdB(&self, x: Num, y: Num) -> Num
    {
        2.0 * self.s * self.B(x,y).powi((2.0 * self.s) as i32 - 1)
    }

    fn dBdb(&self, x: Num, y: Num) -> Num
    {
        -self.C(x,y) / self.b.powi(2)
    }

    pub fn dMdp(&self, x: Num, y: Num) -> Num
    {
        self.dXdp(x,y) + self.dYdp(x,y)
    }

    fn dXdp(&self, x: Num, y: Num) -> Num
    {
        self.dXdA(x,y) * self.dAdp()
    }

    fn dAdp(&self) -> Num
    {
        self.dRdp() / self.a
    }

    fn dRdp(&self) -> Num
    {
        self.dFdp() * self.t.cos() + self.dGdp() * self.t.sin()
    }

    fn dFdp(&self) -> Num
    {
        -1.0
    }

    fn dGdp(&self) -> Num
    {
        0.0
    }

    fn dYdp(&self, x: Num, y: Num) -> Num
    {
        self.dYdB(x,y) * self.dBdp()
    }

    fn dBdp(&self) -> Num
    {
        self.dCdp() / self.b
    }

    fn dCdp(&self) -> Num
    {
        self.dGdp() + self.dFdp()
    }

    pub fn dMdq(&self, x: Num, y: Num) -> Num
    {
        self.dXdq(x,y) + self.dYdq(x,y)
    }

    fn dXdq(&self, x: Num, y: Num) -> Num
    {
        self.dXdA(x,y) * self.dAdq()
    }

    fn dAdq(&self) -> Num
    {
        self.dRdq() / self.a
    }

    fn dRdq(&self) -> Num
    {
        self.dFdq() * self.t.cos() + self.dGdq() * self.t.sin()
    }

    fn dFdq(&self) -> Num
    {
        0.0
    }

    fn dGdq(&self) -> Num
    {
        -1.0
    }

    fn dYdq(&self, x: Num, y: Num) -> Num
    {
        self.dYdb(x,y) * self.dBdq()
    }

    fn dBdq(&self) -> Num
    {
        self.dCdq() / self.b
    }

    fn dCdq(&self) -> Num
    {
        self.dGdq() * self.t.cos() - self.dFdq() * self.t.sin()
    }

    pub fn dMds(&self, x: Num, y: Num) -> Num
    {
        self.dXds(x,y) + self.dYds(x,y)
    }

    fn dXds(&self, x: Num, y: Num) -> Num
    {
        self.A(x,y).powi(2).ln() * self.X(x,y)
    }

    fn dYds(&self, x: Num, y: Num) -> Num
    {
        self.B(x,y).powi(2).ln() * self.Y(x,y)
    }

    pub fn dMdt(&self, x: Num, y: Num) -> Num
    {
        self.dXdt(x,y) + self.dYdt(x,y)
    }

    fn dXdt(&self, x: Num, y: Num) -> Num
    {
        self.dXdA(x,y) * self.dAdt(x,y)
    }

    fn dAdt(&self, x: Num, y: Num) -> Num
    {
        self.dRdt(x,y) / self.a
    }

    fn dRdt(&self, x: Num, y: Num) -> Num
    {
        self.F(x) * -self.t.sin() + self.G(y) * self.t.cos()
    }

    fn dYdt(&self, x: Num, y: Num) -> Num
    {
        self.dYdB(x,y) * self.dBdt(x,y)
    }

    fn dBdt(&self, x: Num, y: Num) -> Num
    {
        self.dCdt(x,y) / self.b
    }

    fn dCdt(&self, x: Num, y: Num) -> Num
    {
        self.G(y) * -self.t.sin() - self.F(x) * -self.t.cos()
    }

    // pub fn fit(&mut self, points: &Points, gamma: Num)
    // {
    //     let (dJda, dJdb, dJdp, dJdq, dJdt) = points.par_iter().map(|p|
    //     {
    //         let loss = self.M(p.0, p.1) - 1.0;

    //         let dJda = self.dMda(p.0, p.1) * loss;
    //         let dJdb = self.dMdb(p.0, p.1) * loss;
    //         let dJdp = self.dMdp(p.0, p.1) * loss;
    //         let dJdq = self.dMdq(p.0, p.1) * loss;
    //         let dJdt = self.dMdt(p.0, p.1) * loss;

    //         (dJda, dJdb, dJdp, dJdq, dJdt)
    //     })
    //     .reduce(|| (0.0, 0.0, 0.0, 0.0, 0.0), |a,b|
    //     {
    //         (
    //             a.0 + b.0,
    //             a.1 + b.1,
    //             a.2 + b.2,
    //             a.3 + b.3,
    //             a.4 + b.4,
    //         )
    //     });

    //     self.a -= gamma*dJda;
    //     self.b -= gamma*dJdb;
    //     self.p -= gamma*dJdp;
    //     self.q -= gamma*dJdq;
    //     self.t -= gamma*dJdt;
    // }

    pub fn fit(&mut self, points: &Points, gamma: Num)
    {
        let step = 0.001;

        let current_loss = self.loss(points);

        self.a += step;
        let dJda = (self.loss(points) - current_loss) / step;
        self.a -= step;

        self.b += step;
        let dJdb = (self.loss(points) - current_loss) / step;
        self.b -= step;

        self.p += step;
        let dJdp = (self.loss(points) - current_loss) / step;
        self.p -= step;

        self.q += step;
        let dJdq = (self.loss(points) - current_loss) / step;
        self.q -= step;

        self.t += step;
        let dJdt = (self.loss(points) - current_loss) / step;
        self.t -= step;

        println!("Changes: {:?}", (gamma*dJda, gamma*dJdb, gamma*dJdp, gamma*dJdq, gamma*dJdt));

        self.a -= gamma * dJda;
        self.b -= gamma * dJdb;
        self.p -= gamma * dJdp;
        self.q -= gamma * dJdq;
        self.t -= gamma * dJdt;
    }


    pub fn loss(&self, points: &Points) -> Num
    {
        points.par_iter().map(|p|
        {
            let ji = 0.5 * self.M(p.0, p.1).powi(2);
            println!("point: {:?} loss: {}", p, ji);
            ji
        })
        .sum()
    }
}
