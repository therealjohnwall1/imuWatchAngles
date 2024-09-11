#define _GNU_SOURCE
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

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

void run_one_sim(int num_steps, int save_period, struct rotor jerk) {
  double Δt = 0.01;

  struct rotor current_rotation = {.s = 1.0, .yz = 0.0, .zx = 0.0, .xy = 0.0};

  struct rotor current_vel = {.s = 1.0, .yz = 0.0, .zx = 0.0, .xy = 0.0};

  struct rotor current_accel = {.s = 1.0, .yz = 0.0, .zx = 0.0, .xy = 0.0};

  for (int i = 0; i < num_steps; i++) {
    if (i % save_period == 0) {
      printf(",%f,%f,%f,%f", current_vel.s, current_vel.yz, current_vel.zx,
             current_vel.xy);
    }

    double deviation1 = (double)random() / (double)RAND_MAX;
    double deviation2 = (double)random() / (double)RAND_MAX;
    double deviation3 = (double)random() / (double)RAND_MAX;
    double deviation4 = (double)random() / (double)RAND_MAX;
    double deviation5 = (double)random() / (double)RAND_MAX;
    double deviation6 = (double)random() / (double)RAND_MAX;
    double deviation7 = (double)random() / (double)RAND_MAX;

    struct rotor jerk_random = make_rotor(
        (struct vector){
            .x = deviation1,
            .y = deviation2,
            .z = deviation3,
        },
        (struct vector){
            .x = deviation4,
            .y = deviation5,
            .z = deviation6,
        },
        0.01 * (1.0 - (2 * deviation7)));

    jerk = rotor_compose(jerk_random, jerk);
    current_accel = rotor_compose(percentage(jerk, Δt), current_accel);
    current_vel = rotor_compose(percentage(current_accel, Δt), current_vel);
    current_rotation =
        rotor_compose(percentage(current_vel, Δt), current_rotation);
  }
  printf("\n");
}

int main() {
  srand(time(NULL)); // randomize seed

  struct rotor classes_start_jerk[] = {
      make_rotor(
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
          M_2_PI),
      make_rotor(
          (struct vector){
              .x = 0.5,
              .y = 0.5,
              .z = 0.0,
          },
          (struct vector){
              .x = 1.0,
              .y = 0.0,
              .z = 0.0,
          },
          M_2_PI),
      make_rotor(
          (struct vector){
              .x = 2.0,
              .y = 9.0,
              .z = 0.0,
          },
          (struct vector){
              .x = 0.0,
              .y = 0.0,
              .z = 1.0,
          },
          M_2_PI),
      make_rotor(
          (struct vector){
              .x = 3.0,
              .y = 3.0,
              .z = 3.0,
          },
          (struct vector){
              .x = 1.0,
              .y = -1.0,
              .z = 2.0,
          },
          M_2_PI),
  };

  int num_classes = sizeof(classes_start_jerk) / sizeof(classes_start_jerk[0]);

  int num_steps = 500;
  int save_period = 50;

  int num_saves = num_steps / save_period;

  // write header
  printf("label");
  for (int i = 0; i < num_saves; i++) {
    printf(",s_%d,i_%d,j_%d,k_%d", i, i, i, i);
  }
  printf("\n");

  for (int class = 0; class < num_classes; class++) {
    for (int i = 0; i < 3; i++) {
      printf("%d", class);
      run_one_sim(num_steps, save_period, classes_start_jerk[class]);
    }
  }
}
