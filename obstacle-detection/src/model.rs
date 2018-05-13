use ::Point;
use ::Points;

pub struct Model
{
    pub a: f32,
    pub b: f32,
    pub theta: f32,
    pub s: i32,
}

impl Model
{
    pub fn fit(points: Points, gamma: f32) -> Self
    {
        let self = Model
        {
            a: 1.0,
            b: 1.0,
            theta: 0.0,
            s: 50,
        };

        loop
        {
            let (dJda, dJdb, dJdt, dJds) = self.gradient_descent(&points);

            self.a     = self.a     - gamma*dJda;
            self.b     = self.b     - gamma*dJdb;
            self.theta = self.theta - gamma*dJdt;
            self.s     = self.s     - gamma*dJds;

            let change = (dJda.powi(2) + dJdb.powi(2) + dJdt.powi(2) + dJds.powi(2)).sqrt();

            if change < 0.02 { break; }
        }

        self
    }

    fn gradient_descent(&self, points: &Points) -> Gradients
    {
        let (st, ct) = self.theta.sin_cos();

        let (dJda, dJdb, dJdt, dJds) = points.par_iter()
        .map(|p|
        {
            let a = (p.0 * ct + p.1 * st);
            let b = (p.1 * ct - p.0 * st);

            let ap = (p.1 * ct - p.0 * st);
            let bp = (p.1 * st + p.0 * ct);

            let A = (a / self.a).powi(2 * self.s);
            let B = (b / self.b).powi(2 * self.s);

            let M = A + B;
            let dMda = -2 * (self.s as f32 / self.a) * A;
            let dMdb = -2 * (self.s as f32 / self.b) * B;
            let dMdt = 2 * self.s * ((ap / self.a) - (bp / self.b));
            let dMds = (self.s as f32).recip() * (A*A.ln() + B*B.ln());

            let dJ = -2 * (1 - M);

            let dJda = dJ * dMda;
            let dJdb = dJ * dMdb;
            let dJdt = dJ * dMdt;
            let dJds = dJ * dMds;

            (dJda, dJdb, dJdt, dJds)
        })
        .reduce(|| (0, 0, 0, 0), |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2, a.3 + b.3));
    }
}

struct Gradients
{
    dJda: f32,
    dJdb: f32,
    dJdt: f32,
    dJds: f32,
}
