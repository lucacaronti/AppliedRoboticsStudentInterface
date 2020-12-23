#include "voronoi.hpp"

Point resize(Point p){
   int minX = -15, maxX = 15;
   int minY = -15, maxY = 15;

   int xDiff = maxX - minX;
   int yDiff = maxY - minY;

   double ratioX = 800/xDiff;
   double ratioY = 800/yDiff;  

   Point p_opencv;
   p_opencv.x = (p.x-minX)*ratioX;
   p_opencv.y = (maxY-p.y)*ratioY;

   return(p_opencv);
}


void plot(std::vector<point_type> points, std::vector<segment_type> segments, const voronoi_diagram<double> &vd){

   Point p1, p2;

   // plot points obstacles
   for(int i = 0; i < points.size(); i++){
         p1 = Point(points[i].x(), points[i].y());
         circle(image, resize(p1), 2, Scalar(0,0,255), -1, LINE_AA );
   }

   // plot segments obstacles
   for(int i = 0; i < segments.size(); i++){
         p1 = Point(low(segments[i]).x(), low(segments[i]).y());
         p2 = Point(high(segments[i]).x(), high(segments[i]).y());
         line(image, resize(p1), resize(p2), Scalar(0, 0, 255), 2, LINE_AA);
   }


   /*   plot edges found in construct_voronoi. 3 cases:
    *   1 - straight finite segments
    *   2 - straight infinite segments (clipped to the border)
    *   3 - curved arcs
    */
   for(const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it) {
      if(!it->is_primary()){
         continue;
      }
      vector<point_type> samples;
      vector<Point> edges;
      
      if(!it->is_finite()){
         clip_infinite_edge(*it, &samples);
      }
      else{
         point_type vertex0(it->vertex0()->x(), it->vertex0()->y());
         samples.push_back(vertex0);
         point_type vertex1(it->vertex1()->x(), it->vertex1()->y());
         samples.push_back(vertex1);

         if(it->is_curved()){
            sample_curved_edge(*it, &samples);
         }
      }

      // convert voronoi points in cvPoint to plot
      for(int i = 0; i < samples.size(); i++){
         p1 = Point(samples[i].x(), samples[i].y());
         edges.push_back(resize(p1));
      }

      polylines(image, edges, false, Scalar(0, 0, 0), 2, LINE_AA );
      
   }

   char window[] = "Voronoi";
   imshow(window, image);
   moveWindow(window, 0, 0);
   waitKey();

}

void clip_infinite_edge(const edge_type& edge, vector<point_type>* clipped_edge){
   const cell_type& cell1 = *edge.cell();
   const cell_type& cell2 = *edge.twin()->cell();
   point_type origin, direction;
   if (cell1.contains_point() && cell2.contains_point()){
      point_type p1 = retrieve_point(cell1);
      point_type p2 = retrieve_point(cell2);
      origin.x((p1.x() + p2.x()) * 0.5);
      origin.y((p1.y() + p2.y()) * 0.5);
      direction.x(p1.y() - p2.y());
      direction.y(p2.x() - p1.x());
   } else {
      origin = cell1.contains_segment() ? 
         retrieve_point(cell2) :
         retrieve_point(cell1);
      segment_type segment = cell1.contains_segment() ?
         retrieve_segment(cell1) :
         retrieve_segment(cell2);
      double dx = high(segment).x() - low(segment).x();
      double dy = high(segment).y() - low(segment).y();
      if ((low(segment) == origin) ^ cell1.contains_point()) {
         direction.x(dy);
         direction.y(-dx);
      } else {
         direction.x(-dy);
         direction.y(dx);
      }
   }
   double side = 15;
   double koef = side / (max)(fabs(direction.x()), fabs(direction.y()));
   if (edge.vertex0() == NULL) {
      clipped_edge->push_back(point_type(
          origin.x() - direction.x() * koef,
          origin.y() - direction.y() * koef));
    } else {
      clipped_edge->push_back(
          point_type(edge.vertex0()->x(), edge.vertex0()->y()));
    }
    if (edge.vertex1() == NULL) {
      clipped_edge->push_back(point_type(
          origin.x() + direction.x() * koef,
          origin.y() + direction.y() * koef));
    } else {
      clipped_edge->push_back(
          point_type(edge.vertex1()->x(), edge.vertex1()->y()));
    }
  }

void sample_curved_edge( const edge_type& edge, vector<point_type>* sampled_edge) {
   
   double max_dist = 1E-3 * 800;
   point_type point = edge.cell()->contains_point() ?
      retrieve_point(*edge.cell()) :
      retrieve_point(*edge.twin()->cell());
   segment_type segment = edge.cell()->contains_point() ?
      retrieve_segment(*edge.twin()->cell()) :
      retrieve_segment(*edge.cell());

   boost::polygon::voronoi_visual_utils<double>::discretize(point, segment, max_dist, sampled_edge);

}

point_type retrieve_point(const cell_type& cell) {
   source_index_type index = cell.source_index();
   source_category_type category = cell.source_category();
   if (category == boost::polygon::SOURCE_CATEGORY_SINGLE_POINT) {
     return points[index];
   }
   index -= points.size();
   if (category == boost::polygon::SOURCE_CATEGORY_SEGMENT_START_POINT) {
     return low(segments[index]);
   } else {
     return high(segments[index]);
   }
}

segment_type retrieve_segment(const cell_type& cell) {
   source_index_type index = cell.source_index() - points.size();
   return segments[index];
}


////////////////////////////////////////////////////////////////////////////

int main()
{

   voronoi_diagram<double> vd;

   points.push_back(point_type(0, 0));
   points.push_back(point_type(1, 6));

   segments.push_back(segment_type(point_type(-4, 5), point_type(5, -1)));
   segments.push_back(segment_type(point_type(3, -11), point_type(13, -1)));

   construct_voronoi(points.begin(), points.end(), segments.begin(), segments.end(), &vd);


   plot(points, segments, vd);

   return 0;
}
