# Dubins

```c++
class DubinsCurve{
private:
    double x0, y0, th0;
    double xf, yf, thf;
    vector<pair<int ,curve> > multi_curve;
    vector<Point2d> middle_points;
    double kmax;
public:
  DubinsCurve();
  ~DubinsCurve();
  void set_k(double _k);
  void add_start_data(double x0, double y0, double th0);
  void add_final_data(double xf, double yf, double thf);
  void add_middle_points(double px, double py);
  tuple<bool, double> solver(int m, int k); 
  void plot() const;
  Path computePath();
};
```

Struct used:
```c++
struct arc {
    double x0;
    double y0;
    double th0;
    double xf;
    double yf;
    double thf;
    double k;
    double L;
};

struct curve {
    arc a1;
    arc a2;
    arc a3;
    double L;
};
```

---

#### void DubinsCurve::set_k(double _k);

Set the maximum path curvature `kmax`

##### Parameters
* `double _k [in]` value of curvature path to set

---

#### void DubinsCurve::add_start_data(double _x0, double _y0, double _th0);

Set initial configuration of position and angle.

##### Parameters
* `double _x0 [in]` initial x coordinate
* `double _y0 [in]` initial y coordinate
* `double _th0 [in]` initial angle

---

#### void DubinsCurve::add_final_data(double _xf, double _yf, double _thf);

Set final configuration of position and angle.

##### Parameters
* `double _xf [in]` final x coordinate
* `double _yf [in]` final y coordinate
* `double _thf [in]` final angle

---

#### void DubinsCurve::add_middle_points(double px, double py);

Add more points to be reached by dubins curve.

##### Parameters
* `double px [in]` x coordinate
* `double py [in]` y coordinate

---

#### tuple<bool, double> DubinsCurve::solver(int m, int k);

Solve the multipoint dubins problem using Iterative Dynamic Programming.

##### Parameters
* `int m [in]` number of refinement steps
* `int k [in]` granularity of the angle discretisation

##### Return
* `tuple<bool, double>` 
    - `bool` false if an optimal curve is not found, true otherwise
    - `double` length of the optimal curve

##### Description
* Compute at every refinement step:
    - call function `L` starting from the initial point to iteratively compute the length `l_value` of the multicurve
* If one curve was not found, return `<false, l_value>`. Otherwise `<true, l_value>`

--- 

#### double L(int j, int n, const vector<Point2d>& middle_points, double x0, double y0, double th0, double xf, double yf, double thf, double kmax,vector<pair<int ,curve> >& multi_curve ,vector<pair<int ,curve> >& tmp_multi_curve , int m, int k);

Gives the length of the solution, computed in iteratively.

##### Parameters
* `int j [in]` current point 
* `int n [in]` total number of intermediate points
* `const vector<Point2d>& middle_points [in]` all the middle points
* `double x0`, `double y0`, `double th0` `[in]` configuration of the initial point
* `double xf`, `double yf`, `double yf` `[in]` configuration of the final point
* `double kmax [in]` maximum path curvature
* `vector<pair<int ,curve> >& multi_curve [in/out]` vector of all curves found
* `vector<pair<int ,curve> >& tmp_multi_curve [in/out]` temporary vector of curves, used in iterations
* `int m [in]` number of refinement steps
* `int k [in]` granularity of the angle discretisation

##### Return
* `double` length of the solution

##### Description
* If there are no intermediate points:
    - call `minD_j` to compute the the curve from initial to final point
* Else:
    - If it is the last intermediate point:
        - generate angles depending on the value `m`
        - call `minD_j` to compute the the curve from this point to the final point
        - `return`
    - Call `L` considering the next point `j+1` and save the length in `tmp_length`
    - If it is the first intermediate point:
        -  call `minD_j` to compute the the curve from the initial point to this point and sum `tmp_length` computed before
        - `return`
    - Else:
        - generate angles depending on the value `m`
        - call `minD_j` to compute the the curve from the previous point to this point and sum `tmp_length` computed before
        - `return`
        
---

#### double minD_j(double x0,double y0, double xf, double yf, double k, const vector<double>& th0s, double thf,  vector<pair<int ,curve> >& multi_curve )

Compute the distance of subset path. Save also the found curves.

##### Parameters
* `double x0`, `double y0` `[in]` initial point coordinates 
* `double xf`, `double yf` `[in]` final point coordinates
* `double k [in]` maximum path curvature
* `const vector<double>& th0s [in]` vector of all the angles to try
* `double thf [in]` final angle
* `vector<pair<int ,curve> >& multi_curve [in/out]` vector of all curves found

##### Return
* `double` length of the sub-solution

##### Description
* For every every angle in `th0s`:
    - Call `dubins_shortest_path` to solve a single dubins problem
    - Save the best curve found 
* Append the best curve to `multi_curve`

---

#### Path DubinsCurve::computePath();
Translate `multi_curve` to `Path`.

#### Return
* `Path` vector of poses with the positions of the robot

#### Description
* For every curve in `multi_curve`:
    - Retrieve a set a points for each arc depending on the length of the curve
    - Append them to the final `Path` as `Pose` object

---