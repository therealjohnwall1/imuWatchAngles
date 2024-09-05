#define _GNU_SOURCE
#include <math.h>
#include <stdio.h>

struct rotor {
  double s;
  double yz;
  double zx;
  double xy;
};

struct bivec {
  double yz;
  double zx;
  double xy;
};

struct vector {
  double x;
  double y;
  double z;
};

struct rotor rotor_compose(struct rotor r1, struct rotor r2) {
  // copied from https://jacquesheunis.com/post/rotors/
  return (struct rotor){
      .s = (r1.s * r2.s) - (r1.xy * r2.xy) - (r1.yz * r2.yz) - (r1.zx * r2.zx),
      .xy = (r1.s * r2.xy) + (r1.xy * r2.s) - (r1.yz * r2.zx) + (r1.zx * r2.yz),
      .yz = (r1.s * r2.yz) + (r1.xy * r2.zx) + (r1.yz * r2.s) - (r1.zx * r2.xy),
      .zx = (r1.s * r2.zx) - (r1.xy * r2.yz) + (r1.yz * r2.xy) + (r1.zx * r2.s),
  };
}

struct rotor rotor_scale_multiply(double scalar, struct rotor r) {
  return (struct rotor){
      .s = scalar * r.s,
      .yz = scalar * r.yz,
      .zx = scalar * r.zx,
      .xy = scalar * r.xy,
  };
}

double dot_product(struct rotor r1, struct rotor r2) {
  return (r1.s * r2.s) - (r1.yz * r2.yz) - (r1.zx * r2.zx) - (r1.xy * r2.xy);
}

double rotor_len_squared(struct rotor r1) {
  return (r1.s * r1.s) + (r1.yz * r1.yz) + (r1.zx * r1.zx) + (r1.xy * r1.xy);
}

struct vector rotor_apply(struct rotor r, struct vector v) {
  // copied from https://jacquesheunis.com/post/rotors/

  double S_x = r.s * v.x + r.xy * v.y - r.zx * v.z;
  double S_y = r.s * v.y - r.xy * v.x + r.yz * v.z;
  double S_z = r.s * v.z - r.yz * v.y + r.zx * v.x;
  double S_xyz = r.xy * v.z + r.yz * v.x + r.zx * v.y;

  double mag_squared = rotor_len_squared(r);

  return (struct vector){
      .x = (S_x * r.s + S_y * r.xy + S_xyz * r.yz - S_z * r.zx) / mag_squared,
      .y = (S_y * r.s - S_x * r.xy + S_z * r.yz + S_xyz * r.zx) / mag_squared,
      .z = (S_z * r.s + S_xyz * r.xy - S_y * r.yz + S_x * r.zx) / mag_squared,
  };
}

double bivec_mag(struct bivec b) {
  return sqrt((b.yz * b.yz) + (b.zx * b.zx) + (b.xy * b.xy));
}

struct rotor make_rotor(struct vector from_vec, struct vector to_vec,
                        double θ_radians) {
  double sin_θ = sin(θ_radians / 2.0);
  double cos_θ = cos(θ_radians / 2.0);

  struct bivec bivec = {
      .yz = (to_vec.y * from_vec.z) - (to_vec.z * from_vec.y),
      .zx = (to_vec.z * from_vec.x) - (to_vec.x * from_vec.z),
      .xy = (to_vec.x * from_vec.y) - (to_vec.y * from_vec.x),
  };

  double mag = bivec_mag(bivec);

  return (struct rotor){
      .s = cos_θ,
      .yz = sin_θ * bivec.yz / mag,
      .zx = sin_θ * bivec.zx / mag,
      .xy = sin_θ * bivec.xy / mag,
  };
}

struct rotor rotor_exponentiation(struct rotor r) {
  // https://math.stackexchange.com/questions/162863/how-to-get-a-part-of-a-quaternion-e-g-get-half-of-the-rotation-of-a-quaternion
  // https://math.stackexchange.com/questions/939229/unit-quaternion-to-a-scalar-power

  struct bivec b = {
      .yz = r.yz,
      .zx = r.zx,
      .xy = r.xy,
  };

  double b_mag = bivec_mag(b);

  return rotor_scale_multiply(exp(r.s), (struct rotor){
                                            .s = cos(b_mag),
                                            .yz = sin(b_mag) * r.yz / b_mag,
                                            .zx = sin(b_mag) * r.zx / b_mag,
                                            .xy = sin(b_mag) * r.xy / b_mag,
                                        });
}

struct rotor rotor_ln(struct rotor r) {
  // https://math.stackexchange.com/questions/162863/how-to-get-a-part-of-a-quaternion-e-g-get-half-of-the-rotation-of-a-quaternion
  // https://math.stackexchange.com/questions/939229/unit-quaternion-to-a-scalar-power

  struct bivec b = {
      .yz = r.yz,
      .zx = r.zx,
      .xy = r.xy,
  };
  double b_mag = bivec_mag(b);
  double rotor_mag = sqrt(rotor_len_squared(r));

  double ϕ = acos(r.s / rotor_mag);

  return (struct rotor){
      .s = log(rotor_mag),
      .yz = ϕ * r.yz / b_mag,
      .zx = ϕ * r.zx / b_mag,
      .xy = ϕ * r.xy / b_mag,
  };
}

struct rotor percentage(struct rotor r, double n) {
  // https://math.stackexchange.com/questions/162863/how-to-get-a-part-of-a-quaternion-e-g-get-half-of-the-rotation-of-a-quaternion
  // https://math.stackexchange.com/questions/939229/unit-quaternion-to-a-scalar-power

  return rotor_exponentiation(rotor_scale_multiply(n, rotor_ln(r)));
}

int main() {
  struct rotor current_rotation = {.s = 1.0, .yz = 0.0, .zx = 0.0, .xy = 0.0};

  struct rotor current_vel = {.s = 1.0, .yz = 0.0, .zx = 0.0, .xy = 0.0};

  struct rotor current_accel = {.s = 1.0, .yz = 0.0, .zx = 0.0, .xy = 0.0};

  /* struct rotor jerk = { */
  /*     .s = 0.0, .yz = 0.5, .zx = 0.5, .xy = 0.7071067811865476}; */

  int num_steps = 100;
  double Δt = 0.01;

  struct rotor jerk = make_rotor(
      (struct vector){
          .x = 0.0,
          .y = 1.0,
          .z = 0.0,
      },
      (struct vector){
          .x = 1.0,
          .y = 0.0,
          .z = 0.0,
      },
      M_2_PI);

  for (int i = 0; i < num_steps; i++) {
    // if (i % 10 == 0) {
      printf("%f,%f,%f,%f\n", current_vel.s, current_vel.yz, current_vel.zx,
             current_vel.xy);
    // }
    current_accel = rotor_compose(percentage(jerk, Δt), current_accel);
    current_vel = rotor_compose(percentage(current_accel, Δt), current_vel);
    current_rotation =
        rotor_compose(percentage(current_vel, Δt), current_rotation);
  }
}