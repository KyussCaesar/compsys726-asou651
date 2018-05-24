type Point  = (f32, f32);
type Points = Vec<Point>;

#[derive(Debug)]
pub struct Model
{
    pub a: f64,
    pub b: f64,
    pub p: f64,
    pub q: f64,
    pub theta: f64,
    pub s: f64,
}

impl Model
{
    pub fn fit(
        points: &Points, 
        gamma: f64,
        a:     f64,
        b:     f64,
        p:     f64,
        q:     f64,
        theta: f64) -> Self
    {
        let mut this = Model
        {
            a,
            b,
            p,
            q,
            theta,
            s: 1.0,
        };

        ros_info!("Fitting model starting from {:?}", this);

        loop
        {
            let (dJda, dJdb, dJdp, dJdq, dJdt, dJds) = this.gradients(&points);

            this.a     = this.a     - gamma*dJda;
            this.b     = this.b     - gamma*dJdb;
            this.p     = this.p     - gamma*dJdp;
            this.q     = this.q     - gamma*dJdq;
            this.theta = this.theta - gamma*dJdt;
            this.s     = this.s     - gamma*dJds;

            let change = (dJda, dJdb, dJdp, dJdq, dJdt, dJds);

            // ros_info!("Model deltas: {:?}", change);
            // ros_info!("{:?}", this);

            let change =
            (
                change.0.powi(2) +
                change.1.powi(2) +
                change.2.powi(2) +
                change.3.powi(2) +
                change.4.powi(2) +
                change.5.powi(2)
            ).sqrt();

            // ros_info!("change: {}", change);

            if change < 0.005 { break; }
        }

        this
    }

    fn model(&self, p: &Point) -> f64
    {
        let (st, ct) = self.theta.sin_cos();

        let x = p.0 as f64 - self.p;
        let y = p.1 as f64 - self.q;

        let R = (x * ct + y * st) / self.a;
        let C = (y * ct - x * st) / self.b;

        let s = 2.0 * self.s.round();

        return R.powf(s) + C.powf(s);
    }

    fn loss(&self, p: &Point) -> f64
    {
        (1.0 - self.model(p)).powi(2) / 2.0
    }

    fn gradients(&mut self, points: &Points) -> (f64, f64, f64, f64, f64, f64)
    {
        points.iter()
        .map(|p|
        {
            let step = 0.001;
            let current_val = self.loss(p);

            self.a += step;
            let dJda = (self.loss(p) - current_val) / step;
            self.a -= step;

            self.b += step;
            let dJdb = (self.loss(p) - current_val) / step;
            self.b -= step;

            self.p += step;
            let dJdp = (self.loss(p) - current_val) / step;
            self.p -= step;

            self.q += step;
            let dJdq = (self.loss(p) - current_val) / step;
            self.q -= step;

            self.theta += step;
            let dJdt = (self.loss(p) - current_val) / step;
            self.theta -= step;

            self.s += step;
            let dJds = (self.loss(p) - current_val) / step;
            self.s -= step;

            (dJda, dJdb, dJdp, dJdq, dJdt, dJds)
        })
        .fold((0.0, 0.0, 0.0, 0.0, 0.0, 0.0), |acc, x|
        {
            (
                acc.0 + x.0, 
                acc.1 + x.1,
                acc.2 + x.2,
                acc.3 + x.3,
                acc.4 + x.4,
                acc.5 + x.5,
            )
        })
    }

    // {
    //     ros_info!("Running one iteration of gradient descent");

    //     let (st, ct) = self.theta.sin_cos();

    //     ros_info!("sin {} cos {}", st, ct);

    //     points.par_iter()
    //     .map(|p|
    //     {
    //         ros_info!("Point: {:?}", p);

    //         let a = p.0 as f64 * ct + p.1 as f64 * st;
    //         let b = p.1 as f64 * ct - p.0 as f64 * st;

    //         ros_info!("a: {}, b: {}", a, b);

    //         let ap = p.1 as f64 * ct - p.0 as f64 * st;
    //         let bp = p.1 as f64 * st + p.0 as f64 * ct;

    //         ros_info!("ap: {}, bp: {}", ap, bp);

    //         let A = (a / self.a).powf(2.0 * self.s.round());
    //         let B = (b / self.b).powf(2.0 * self.s.round());

    //         ros_info!("A: {}, B: {}", A, B);

    //         let M = A + B;
    //         let dMda = -2.0 * (self.s.round() / self.a) * A;
    //         let dMdb = -2.0 * (self.s.round() / self.b) * B;
    //         let dMdt =  2.0 *  self.s.round() * ((ap / self.a) - (bp / self.b));
    //         let dMds = self.s.round().recip() * (A*A.ln() + B*B.ln());

    //         ros_info!("dMda {}", dMda);
    //         ros_info!("dMdb {}", dMdb);
    //         ros_info!("dMdt {}", dMdt);
    //         ros_info!("dMds {}", dMds);

    //         let dJ = -2.0 * (1.0 - M);

    //         let dJda = dJ * dMda;
    //         let dJdb = dJ * dMdb;
    //         let dJdt = dJ * dMdt;
    //         let dJds = dJ * dMds;

    //         ros_info!("dJda {}", dJda);
    //         ros_info!("dJdb {}", dJdb);
    //         ros_info!("dJdt {}", dJdt);
    //         ros_info!("dJds {}", dJds);

    //         (dJda, dJdb, dJdt, dJds)
    //     })
    //     .reduce(|| (0.0, 0.0, 0.0, 0.0), |a, b| (a.0 + b.0, a.1 + b.1, a.2 + b.2, a.3 + b.3))
    // }
}
