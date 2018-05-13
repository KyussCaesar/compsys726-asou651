use ::Points;

use rayon::prelude::*;

#[derive(Debug)]
pub struct Model
{
    pub a: f32,
    pub b: f32,
    pub theta: f32,
    pub s: f32,
}

impl Model
{
    pub fn fit(points: &Points, gamma: f32, a: f32, b: f32, theta: f32) -> Self
    {
        let mut this = Model
        {
            a,
            b,
            theta,
            s: 50.0,
        };

        loop
        {
            let (dJda, dJdb, dJdt, dJds) = this.gradient_descent(&points);

            this.a     = this.a     - gamma*dJda;
            this.b     = this.b     - gamma*dJdb;
            this.theta = this.theta - gamma*dJdt;
            this.s     = this.s     - gamma*dJds;

            let change = (dJda.powi(2) + dJdb.powi(2) + dJdt.powi(2) + dJds.powi(2)).sqrt();

            if change < 0.02 { break; }
        }

        this
    }

    fn gradient_descent(&self, points: &Points) -> (f32, f32, f32, f32)
    {
        let (st, ct) = self.theta.sin_cos();

        points.par_iter()
        .map(|p|
        {
            let a = p.0 as f32 * ct + p.1 as f32 * st;
            let b = p.1 as f32 * ct - p.0 as f32 * st;

            let ap = p.1 as f32 * ct - p.0 as f32 * st;
            let bp = p.1 as f32 * st + p.0 as f32 * ct;

            let A = (a / self.a).powf(2.0 * self.s.round());
            let B = (b / self.b).powf(2.0 * self.s.round());

            let M = A + B;
            let dMda = -2.0 * (self.s.round() / self.a) * A;
            let dMdb = -2.0 * (self.s.round() / self.b) * B;
            let dMdt = 2.0 * self.s.round() * ((ap / self.a) - (bp / self.b));
            let dMds = self.s.round().recip() * (A*A.ln() + B*B.ln());

            let dJ = -2.0 * (1.0 - M);

            let dJda = dJ * dMda;
            let dJdb = dJ * dMdb;
            let dJdt = dJ * dMdt;
            let dJds = dJ * dMds;

            (dJda, dJdb, dJdt, dJds)
        })
        .reduce(|| (0.0, 0.0, 0.0, 0.0), |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2, a.3 + b.3))
    }
}
