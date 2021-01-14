# Intersections

Functions used to check if two segments intersect.

__NOTE:__ Taken from https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/

--- 

#### bool doIntersect(Point2d p1, Point2d q1, Point2d p2, Point2d q2);

##### Parameters
* `Point2d p1`, `Point2d q1` `[in]` points that identify the first segment
* `Point2d p2`, `Point2d q2` `[in]` points that identify the second segment

##### Return
* `bool` true if they intersect, false otherwise

##### Description
* Compute the orientation of the four possible triplets
* General case: if both the orientation pairs relative to one segment are different, then the two segments intersect
* Special case: if one triple is collinear and the other point lies on the segment, then the two segments intersect
* If none of the previous cases is met, the two segments do not intersect

---

#### int orientation(Point2d p, Point2d q, Point2d r);

##### Parameters
* `Point2d p`, `Point2d q` points that identify one segment
* `Point2d r` initial or final point of the other segment

##### Return
* `int` 0 if collinear, 1 if clockwise, 2 if counterclockwise

---

#### bool onSegment(Point2d p, Point2d q, Point2d r);
* `Point2d p`, `Point2d r` points that identify one segment
* `Point2d q` initial or final point of the other segment

##### Return
* `bool` true if q lies on segment pr, false otherwise

---