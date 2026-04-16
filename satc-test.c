#include "assert.h"
#include "satc.h"

#define SATC_TEST_EPSILON 0.0001
#define satc_assert_near(actual, expected) assert(fabs((actual) - (expected)) < SATC_TEST_EPSILON)
#define satc_nearest_hundredth(n) floor(n * 100 + 0.5) / 100

void satc_point_scale_xy_test () {
  satc_point_alloca(v);
  satc_point_set_xy(v, 5.0, 5.0);
  satc_point_scale_xy(v, 10.0, 10.0);
  assert(satc_point_get_x(v) == 50.0);
  assert(satc_point_get_y(v) == 50.0);
  satc_point_scale_xy(v, 0.0, 1.0);
  assert(satc_point_get_x(v) == 0.0);
  assert(satc_point_get_y(v) == 50.0);
  satc_point_scale_xy(v, 1.0, 0.0);
  assert(satc_point_get_x(v) == 0.0);
  assert(satc_point_get_y(v) == 0.0);

  satc_point_set_xy(v, 1.0, 1.0);
  satc_point_rotate(v, M_PI / 4.0);
  satc_assert_near(satc_point_get_x(v), 0.0);
  satc_assert_near(satc_point_get_y(v), sqrt(2.0));

  satc_point_set_xy(v, 3.0, 4.0);
  satc_point_alloca_xy(axis, 1.0, 0.0);
  satc_point_project(v, axis);
  satc_assert_near(satc_point_get_x(v), 3.0);
  satc_assert_near(satc_point_get_y(v), 0.0);

  satc_point_set_xy(v, 3.0, 4.0);
  satc_point_reflect(v, axis);
  satc_assert_near(satc_point_get_x(v), 3.0);
  satc_assert_near(satc_point_get_y(v), -4.0);
}

void satc_polygon_get_centroid_test () {
  {
    // Centroid of a square
    satc_point_array_alloca(points, 4);
    satc_point_alloca(pos);
    satc_point_set_xy(pos, 0.0, 0.0);
    satc_point_alloca(a);
    satc_point_set_xy(a, 0.0, 0.0);
    satc_point_alloca(b);
    satc_point_set_xy(b, 40.0, 0.0);
    satc_point_alloca(c);
    satc_point_set_xy(c, 40.0, 40.0);
    satc_point_alloca(d);
    satc_point_set_xy(d, 0.0, 40.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    points[3] = d;
    satc_polygon_t *polygon = satc_polygon_create(pos, 4, points);
    double *centroid = satc_polygon_get_centroid(polygon);
    satc_assert_near(satc_point_get_x(centroid), 20.0);
    satc_assert_near(satc_point_get_y(centroid), 20.0);
    satc_point_destroy(centroid);
    satc_polygon_destroy(polygon);
  }

  {
    // Centroid of a triangle
    satc_point_array_alloca(points, 3);
    satc_point_alloca(pos);
    satc_point_set_xy(pos, 0.0, 0.0);
    satc_point_alloca(a);
    satc_point_set_xy(a, 0.0, 0.0);
    satc_point_alloca(b);
    satc_point_set_xy(b, 100.0, 0.0);
    satc_point_alloca(c);
    satc_point_set_xy(c, 50.0, 99.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    satc_polygon_t *polygon = satc_polygon_create(pos, 3, points);
    double *centroid = satc_polygon_get_centroid(polygon);
    satc_assert_near(satc_point_get_x(centroid), 50.0);
    satc_assert_near(satc_point_get_y(centroid), 33.0);
    satc_point_destroy(centroid);
    satc_polygon_destroy(polygon);
  }

  {
    // Centroid of a degenerate polygon falls back to the average point.
    satc_point_array_alloca(points, 3);
    satc_point_alloca_xy(pos, 0.0, 0.0);
    satc_point_alloca_xy(a, 0.0, 0.0);
    satc_point_alloca_xy(b, 1.0, 1.0);
    satc_point_alloca_xy(c, 2.0, 2.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    satc_polygon_t *polygon = satc_polygon_create(pos, 3, points);
    double *centroid = satc_polygon_get_centroid(polygon);
    assert(isfinite(satc_point_get_x(centroid)));
    assert(isfinite(satc_point_get_y(centroid)));
    satc_assert_near(satc_point_get_x(centroid), 1.0);
    satc_assert_near(satc_point_get_y(centroid), 1.0);
    satc_point_destroy(centroid);
    satc_polygon_destroy(polygon);
  }
}

void satc_collision_test () {
  {
    // Circle-to-circle collision.
    satc_point_alloca_xy(c1_pos, 0.0, 0.0);
    satc_point_alloca_xy(c2_pos, 30.0, 0.0);
    satc_circle_t *circle1 = satc_circle_create(c1_pos, 20.0);
    satc_circle_t *circle2 = satc_circle_create(c2_pos, 20.0);
    satc_response_t *response = satc_response_create();
    bool collided = satc_test_circle_circle(circle1, circle2, response);
    assert(collided);
    assert(satc_point_get_x(response->overlap_v) == 10.0);
    assert(satc_point_get_y(response->overlap_v) == 0.0);
    satc_response_destroy(response);
    satc_circle_destroy(circle2);
    satc_circle_destroy(circle1);
  }

  {
    // Circle-to-circle tangent contact counts as collision.
    satc_point_alloca_xy(c1_pos, 0.0, 0.0);
    satc_point_alloca_xy(c2_pos, 40.0, 0.0);
    satc_circle_t *circle1 = satc_circle_create(c1_pos, 20.0);
    satc_circle_t *circle2 = satc_circle_create(c2_pos, 20.0);
    satc_response_t *response = satc_response_create();
    bool collided = satc_test_circle_circle(circle1, circle2, response);
    assert(collided);
    satc_assert_near(response->overlap, 0.0);
    satc_response_destroy(response);
    satc_circle_destroy(circle2);
    satc_circle_destroy(circle1);
  }

  {
    // Circle-to-circle containment flags.
    satc_point_alloca_xy(c1_pos, 0.0, 0.0);
    satc_point_alloca_xy(c2_pos, 5.0, 0.0);
    satc_circle_t *circle1 = satc_circle_create(c1_pos, 10.0);
    satc_circle_t *circle2 = satc_circle_create(c2_pos, 30.0);
    satc_response_t *response = satc_response_create();
    bool collided = satc_test_circle_circle(circle1, circle2, response);
    assert(collided);
    assert(response->a_in_b);
    assert(!response->b_in_a);
    satc_response_destroy(response);
    satc_circle_destroy(circle2);
    satc_circle_destroy(circle1);
  }

  {
    // Polygon-to-circle collision.
    satc_point_alloca_xy(c_pos, 50.0, 50.0);
    satc_point_alloca_xy(p_pos, 0.0, 0.0);
    satc_point_array_alloca(points, 4);
    satc_point_alloca_xy(a, 0.0, 0.0);
    satc_point_alloca_xy(b, 40.0, 0.0);
    satc_point_alloca_xy(c, 40.0, 40.0);
    satc_point_alloca_xy(d, 0.0, 40.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    points[3] = d;
    satc_circle_t *circle = satc_circle_create(c_pos, 20.0);
    satc_polygon_t *polygon = satc_polygon_create(p_pos, 4, points);
    satc_response_t *response = satc_response_create();
    bool collided = satc_test_polygon_circle(polygon, circle, response);
    assert(collided);
    assert(satc_nearest_hundredth(response->overlap) == 5.86);
    assert(satc_nearest_hundredth(satc_point_get_x(response->overlap_v)) == 4.14);
    assert(satc_nearest_hundredth(satc_point_get_y(response->overlap_v)) == 4.14);
    satc_response_destroy(response);
    satc_circle_destroy(circle);
    satc_polygon_destroy(polygon);
  }

  {
    // Polygon contains circle.
    satc_point_alloca_xy(c_pos, 50.0, 50.0);
    satc_point_alloca_xy(p_pos, 0.0, 0.0);
    satc_point_array_alloca(points, 4);
    satc_point_alloca_xy(a, 0.0, 0.0);
    satc_point_alloca_xy(b, 100.0, 0.0);
    satc_point_alloca_xy(c, 100.0, 100.0);
    satc_point_alloca_xy(d, 0.0, 100.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    points[3] = d;
    satc_circle_t *circle = satc_circle_create(c_pos, 10.0);
    satc_polygon_t *polygon = satc_polygon_create(p_pos, 4, points);
    satc_response_t *response = satc_response_create();
    bool collided = satc_test_polygon_circle(polygon, circle, response);
    assert(collided);
    assert(!response->a_in_b);
    assert(response->b_in_a);
    satc_response_destroy(response);
    satc_circle_destroy(circle);
    satc_polygon_destroy(polygon);
  }

  {
    // Circle inside polygon flips containment flags.
    satc_point_alloca_xy(c_pos, 50.0, 50.0);
    satc_point_alloca_xy(p_pos, 0.0, 0.0);
    satc_point_array_alloca(points, 4);
    satc_point_alloca_xy(a, 0.0, 0.0);
    satc_point_alloca_xy(b, 100.0, 0.0);
    satc_point_alloca_xy(c, 100.0, 100.0);
    satc_point_alloca_xy(d, 0.0, 100.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    points[3] = d;
    satc_circle_t *circle = satc_circle_create(c_pos, 10.0);
    satc_polygon_t *polygon = satc_polygon_create(p_pos, 4, points);
    satc_response_t *response = satc_response_create();
    bool collided = satc_test_circle_polygon(circle, polygon, response);
    assert(collided);
    assert(response->a_in_b);
    assert(!response->b_in_a);
    satc_response_destroy(response);
    satc_circle_destroy(circle);
    satc_polygon_destroy(polygon);
  }

  {
    // Polygon-to-circle collision works without a response object.
    satc_point_alloca_xy(circle_pos, 20.0, 50.0);
    satc_circle_t *circle = satc_circle_create(circle_pos, 20.0);
    satc_point_alloca_xy(polygon_pos, 0.0, 0.0);
    satc_point_array_alloca(polygon_points, 4);
    satc_point_alloca_xy(polygon_point_1, 0.0, 0.0);
    satc_point_alloca_xy(polygon_point_2, 40.0, 0.0);
    satc_point_alloca_xy(polygon_point_3, 40.0, 40.0);
    satc_point_alloca_xy(polygon_point_4, 0.0, 40.0);
    polygon_points[0] = polygon_point_1;
    polygon_points[1] = polygon_point_2;
    polygon_points[2] = polygon_point_3;
    polygon_points[3] = polygon_point_4;
    satc_polygon_t *polygon = satc_polygon_create(polygon_pos, 4, polygon_points);
    bool collided = satc_test_polygon_circle(polygon, circle, NULL);
    assert(collided);
    satc_circle_destroy(circle);
    satc_polygon_destroy(polygon);
  }

  {
    // Polygon-to-polygon collision.
    satc_point_alloca_xy(pos_1, 0.0, 0.0);
    satc_point_alloca_xy(pos_2, 10.0, 0.0);
    satc_box_t *box_1 = satc_box_create(pos_1, 20.0, 20.0);
    satc_polygon_t *polygon_1 = satc_box_to_polygon(box_1);
    satc_box_t *box_2 = satc_box_create(pos_2, 20.0, 20.0);
    satc_polygon_t *polygon_2 = satc_box_to_polygon(box_2);
    satc_response_t *response = satc_response_create();
    bool collided = satc_test_polygon_polygon(polygon_1, polygon_2, response);
    assert(collided);
    satc_assert_near(response->overlap, 10.0);
    satc_assert_near(fabs(satc_point_get_x(response->overlap_v)), 10.0);
    satc_assert_near(satc_point_get_y(response->overlap_v), 0.0);
    satc_response_destroy(response);
    satc_polygon_destroy(polygon_2);
    satc_polygon_destroy(polygon_1);
    satc_box_destroy(box_2);
    satc_box_destroy(box_1);
  }

  {
    // Polygon-to-polygon containment flags.
    satc_point_alloca_xy(pos_1, 20.0, 20.0);
    satc_point_alloca_xy(pos_2, 0.0, 0.0);
    satc_box_t *inner_box = satc_box_create(pos_1, 20.0, 20.0);
    satc_polygon_t *inner_polygon = satc_box_to_polygon(inner_box);
    satc_box_t *outer_box = satc_box_create(pos_2, 100.0, 100.0);
    satc_polygon_t *outer_polygon = satc_box_to_polygon(outer_box);
    satc_response_t *response = satc_response_create();
    bool collided = satc_test_polygon_polygon(inner_polygon, outer_polygon, response);
    assert(collided);
    assert(response->a_in_b);
    assert(!response->b_in_a);
    satc_response_destroy(response);
    satc_polygon_destroy(outer_polygon);
    satc_polygon_destroy(inner_polygon);
    satc_box_destroy(outer_box);
    satc_box_destroy(inner_box);
  }

  {
    // Polygon to polygon no collision.
    satc_point_alloca(pos_1);
    satc_point_set_xy(pos_1, 0.0, 0.0);
    satc_point_alloca(pos_2);
    satc_point_set_xy(pos_2, 100.0, 100.0);
    satc_box_t *box_1 = satc_box_create(pos_1, 20.0, 20.0);
    satc_polygon_t *polygon_1 = satc_box_to_polygon(box_1);
    satc_box_t *box_2 = satc_box_create(pos_2, 20.0, 20.0);
    satc_polygon_t *polygon_2 = satc_box_to_polygon(box_2);
    satc_response_t *response = satc_response_create();
    bool collided = satc_test_polygon_polygon(polygon_1, polygon_2, response);
    assert(!collided);
    satc_response_destroy(response);
    satc_polygon_destroy(polygon_2);
    satc_polygon_destroy(polygon_1);
    satc_box_destroy(box_2);
    satc_box_destroy(box_1);
  }
}

void satc_point_test () {
  {
    // Point in circle
    satc_point_alloca(circle_pos);
    satc_point_set_xy(circle_pos, 100.0, 100.0);
    satc_point_alloca(point_1);
    satc_point_set_xy(point_1, 0.0, 0.0);
    satc_point_alloca(point_2);
    satc_point_set_xy(point_2, 110.0, 110.0);
    satc_circle_t *circle = satc_circle_create(circle_pos, 20.0);
    bool test_1 = satc_point_in_circle(point_1, circle);
    assert(!test_1);
    bool test_2 = satc_point_in_circle(point_2, circle);
    assert(test_2);
    satc_circle_destroy(circle);
  }

  {
    // Point in polygon
    satc_point_alloca(triangle_pos);
    satc_point_set_xy(triangle_pos, 30.0, 0.0);
    satc_point_array_alloca(triangle_points, 3);
    satc_point_alloca(triangle_point_1);
    satc_point_set_xy(triangle_point_1, 0.0, 0.0);
    satc_point_alloca(triangle_point_2);
    satc_point_set_xy(triangle_point_2, 30.0, 0.0);
    satc_point_alloca(triangle_point_3);
    satc_point_set_xy(triangle_point_3, 0.0, 30.0);
    triangle_points[0] = triangle_point_1;
    triangle_points[1] = triangle_point_2;
    triangle_points[2] = triangle_point_3;
    satc_polygon_t *triangle = satc_polygon_create(triangle_pos, 3, triangle_points);
    satc_point_alloca(point_1);
    satc_point_set_xy(point_1, 0.0, 0.0);
    satc_point_alloca(point_2);
    satc_point_set_xy(point_2, 35.0, 5.0);
    bool test_1 = satc_point_in_polygon(point_1, triangle);
    assert(!test_1);
    bool test_2 = satc_point_in_polygon(point_2, triangle);
    assert(test_2);
    satc_polygon_destroy(triangle);
  }

  {
    // Point in polygon (small)

    satc_point_alloca(point);
    satc_point_set_xy(point, 1.0, 1.1);
    satc_point_alloca(polygon_pos);
    satc_point_set_xy(polygon_pos, 0.0, 0.0);
    satc_point_alloca(polygon_point_1);
    satc_point_set_xy(polygon_point_1, 2.0, 1.0);
    satc_point_alloca(polygon_point_2);
    satc_point_set_xy(polygon_point_2, 2.0, 2.0);
    satc_point_alloca(polygon_point_3);
    satc_point_set_xy(polygon_point_3, 1.0, 3.0);
    satc_point_alloca(polygon_point_4);
    satc_point_set_xy(polygon_point_4, 0.0, 2.0);
    satc_point_alloca(polygon_point_5);
    satc_point_set_xy(polygon_point_5, 0.0, 1.0);
    satc_point_alloca(polygon_point_6);
    satc_point_set_xy(polygon_point_6, 1.0, 0.0);
    satc_point_array_alloca(polygon_points, 6);
    polygon_points[0] = polygon_point_1;
    polygon_points[1] = polygon_point_2;
    polygon_points[2] = polygon_point_3;
    polygon_points[3] = polygon_point_4;
    polygon_points[4] = polygon_point_5;
    polygon_points[5] = polygon_point_6;
    satc_polygon_t *polygon = satc_polygon_create(polygon_pos, 6, polygon_points);
    bool test = satc_point_in_polygon(point, polygon);
    assert(test);
    satc_polygon_destroy(polygon);
  }
}

void satc_polygon_transform_test () {
  {
    // Polygon translation adds a delta to every point.
    satc_point_alloca_xy(pos, 0.0, 0.0);
    satc_point_array_alloca(points, 3);
    satc_point_alloca_xy(a, 1.0, 2.0);
    satc_point_alloca_xy(b, 3.0, 4.0);
    satc_point_alloca_xy(c, 5.0, 6.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    satc_polygon_t *polygon = satc_polygon_create(pos, 3, points);
    satc_polygon_translate(polygon, 10.0, 20.0);
    satc_assert_near(satc_point_get_x(polygon->points[0]), 11.0);
    satc_assert_near(satc_point_get_y(polygon->points[0]), 22.0);
    satc_assert_near(satc_point_get_x(polygon->points[1]), 13.0);
    satc_assert_near(satc_point_get_y(polygon->points[1]), 24.0);
    satc_assert_near(satc_point_get_x(polygon->points[2]), 15.0);
    satc_assert_near(satc_point_get_y(polygon->points[2]), 26.0);
    satc_polygon_destroy(polygon);
  }

  {
    // Polygon offset changes calculated points and collision separation.
    satc_point_alloca_xy(pos_1, 0.0, 0.0);
    satc_point_array_alloca(points_1, 4);
    satc_point_alloca_xy(a_1, -10.0, -10.0);
    satc_point_alloca_xy(b_1, 10.0, -10.0);
    satc_point_alloca_xy(c_1, 10.0, 10.0);
    satc_point_alloca_xy(d_1, -10.0, 10.0);
    points_1[0] = a_1;
    points_1[1] = b_1;
    points_1[2] = c_1;
    points_1[3] = d_1;
    satc_polygon_t *polygon_1 = satc_polygon_create(pos_1, 4, points_1);
    satc_point_alloca_xy(offset, 30.0, 0.0);
    satc_polygon_set_offset(polygon_1, offset);

    satc_point_alloca_xy(pos_2, 0.0, 0.0);
    satc_box_t *box_2 = satc_box_create(pos_2, 20.0, 20.0);
    satc_polygon_t *polygon_2 = satc_box_to_polygon(box_2);

    satc_response_t *response = satc_response_create();
    bool collided = satc_test_polygon_polygon(polygon_1, polygon_2, response);
    assert(collided);
    satc_assert_near(satc_point_get_x(polygon_1->calc_points[0]), 20.0);
    satc_assert_near(satc_point_get_y(polygon_1->calc_points[0]), -10.0);
    satc_assert_near(response->overlap, 10.0);

    satc_response_destroy(response);
    satc_polygon_destroy(polygon_2);
    satc_box_destroy(box_2);
    satc_polygon_destroy(polygon_1);
  }

  {
    // AABB uses deterministic minima for its origin.
    satc_point_alloca_xy(pos, 0.0, 0.0);
    satc_point_array_alloca(points, 3);
    satc_point_alloca_xy(a, 1.0, 2.0);
    satc_point_alloca_xy(b, 4.0, 2.0);
    satc_point_alloca_xy(c, 1.0, 5.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    satc_polygon_t *polygon = satc_polygon_create(pos, 0 + 3, points);
    satc_polygon_t *aabb = satc_polygon_get_aabb(polygon);
    satc_assert_near(satc_point_get_x(aabb->pos), 1.0);
    satc_assert_near(satc_point_get_y(aabb->pos), 2.0);
    satc_polygon_destroy(aabb);
    satc_polygon_destroy(polygon);
  }

  {
    // AABB reflects polygon position, offset, and rotation.
    satc_point_alloca_xy(pos, 10.0, 20.0);
    satc_point_array_alloca(points, 4);
    satc_point_alloca_xy(a, -5.0, -10.0);
    satc_point_alloca_xy(b, 5.0, -10.0);
    satc_point_alloca_xy(c, 5.0, 10.0);
    satc_point_alloca_xy(d, -5.0, 10.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    points[3] = d;
    satc_polygon_t *polygon = satc_polygon_create(pos, 4, points);
    satc_point_alloca_xy(offset, 3.0, -4.0);
    satc_polygon_set_offset(polygon, offset);
    satc_polygon_set_angle(polygon, M_PI / 4.0);

    double x_min = satc_point_get_x(polygon->calc_points[0]) + satc_point_get_x(polygon->pos);
    double y_min = satc_point_get_y(polygon->calc_points[0]) + satc_point_get_y(polygon->pos);
    double x_max = x_min;
    double y_max = y_min;
    size_t i = 1;
    for (; i < polygon->num_calc_points; i++) {
      double x = satc_point_get_x(polygon->calc_points[i]) + satc_point_get_x(polygon->pos);
      double y = satc_point_get_y(polygon->calc_points[i]) + satc_point_get_y(polygon->pos);
      if (x < x_min) x_min = x;
      if (x > x_max) x_max = x;
      if (y < y_min) y_min = y;
      if (y > y_max) y_max = y;
    }

    satc_polygon_t *aabb = satc_polygon_get_aabb(polygon);
    satc_assert_near(satc_point_get_x(aabb->pos), x_min);
    satc_assert_near(satc_point_get_y(aabb->pos), y_min);
    satc_assert_near(satc_point_get_x(aabb->points[2]), x_max - x_min);
    satc_assert_near(satc_point_get_y(aabb->points[2]), y_max - y_min);

    satc_polygon_destroy(aabb);
    satc_polygon_destroy(polygon);
  }

  {
    // Rotated polygons still participate correctly in point tests.
    satc_point_alloca_xy(pos, 50.0, 50.0);
    satc_point_array_alloca(points, 4);
    satc_point_alloca_xy(a, -10.0, -10.0);
    satc_point_alloca_xy(b, 10.0, -10.0);
    satc_point_alloca_xy(c, 10.0, 10.0);
    satc_point_alloca_xy(d, -10.0, 10.0);
    points[0] = a;
    points[1] = b;
    points[2] = c;
    points[3] = d;
    satc_polygon_t *polygon = satc_polygon_create(pos, 4, points);
    satc_polygon_set_angle(polygon, M_PI / 4.0);
    satc_point_alloca_xy(point_inside, 50.0, 50.0);
    satc_point_alloca_xy(point_outside, 80.0, 80.0);
    assert(satc_point_in_polygon(point_inside, polygon));
    assert(!satc_point_in_polygon(point_outside, polygon));
    satc_polygon_destroy(polygon);
  }

  {
    // Rotated polygons still collide correctly under SAT.
    satc_point_alloca_xy(pos_1, 0.0, 0.0);
    satc_point_array_alloca(points_1, 4);
    satc_point_alloca_xy(a_1, -10.0, -10.0);
    satc_point_alloca_xy(b_1, 10.0, -10.0);
    satc_point_alloca_xy(c_1, 10.0, 10.0);
    satc_point_alloca_xy(d_1, -10.0, 10.0);
    points_1[0] = a_1;
    points_1[1] = b_1;
    points_1[2] = c_1;
    points_1[3] = d_1;
    satc_polygon_t *polygon_1 = satc_polygon_create(pos_1, 4, points_1);
    satc_polygon_set_angle(polygon_1, M_PI / 4.0);

    satc_point_alloca_xy(pos_2, 6.0, 0.0);
    satc_box_t *box_2 = satc_box_create(pos_2, 20.0, 20.0);
    satc_polygon_t *polygon_2 = satc_box_to_polygon(box_2);

    satc_response_t *response = satc_response_create();
    bool collided = satc_test_polygon_polygon(polygon_1, polygon_2, response);
    assert(collided);
    assert(response->overlap > 0.0);

    satc_response_destroy(response);
    satc_polygon_destroy(polygon_2);
    satc_box_destroy(box_2);
    satc_polygon_destroy(polygon_1);
  }

  {
    // Rotated polygons can still separate under SAT.
    satc_point_alloca_xy(pos_1, 0.0, 0.0);
    satc_point_array_alloca(points_1, 4);
    satc_point_alloca_xy(a_1, -10.0, -10.0);
    satc_point_alloca_xy(b_1, 10.0, -10.0);
    satc_point_alloca_xy(c_1, 10.0, 10.0);
    satc_point_alloca_xy(d_1, -10.0, 10.0);
    points_1[0] = a_1;
    points_1[1] = b_1;
    points_1[2] = c_1;
    points_1[3] = d_1;
    satc_polygon_t *polygon_1 = satc_polygon_create(pos_1, 4, points_1);
    satc_polygon_set_angle(polygon_1, M_PI / 4.0);

    satc_point_alloca_xy(pos_2, 30.0, 0.0);
    satc_box_t *box_2 = satc_box_create(pos_2, 20.0, 20.0);
    satc_polygon_t *polygon_2 = satc_box_to_polygon(box_2);

    satc_response_t *response = satc_response_create();
    bool collided = satc_test_polygon_polygon(polygon_1, polygon_2, response);
    assert(!collided);

    satc_response_destroy(response);
    satc_polygon_destroy(polygon_2);
    satc_box_destroy(box_2);
    satc_polygon_destroy(polygon_1);
  }
}

int main (int argc, char *argv[], char *envp[]) {
  satc_point_scale_xy_test();
  satc_polygon_get_centroid_test();
  satc_collision_test();
  satc_point_test();
  satc_polygon_transform_test();
  return EXIT_SUCCESS;
}
