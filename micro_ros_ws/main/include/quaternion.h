#ifndef QUATERNION_H
#define QUATERNION_H

typedef struct {
    float w;
    float x;
    float y;
    float z;
} quaternion_t;

quaternion_t quaternion_multiply(
    quaternion_t a,
    quaternion_t b
);

quaternion_t quaternion_conjugate(quaternion_t q);

// Devuelve 0 si pudo normalizar; -1 si el valor es inválido.
int quaternion_normalize(quaternion_t *q);

int quaternion_integrate_gyro(quaternion_t *q, const float gyro[3], float dt);

// q debe ser unitario y representar la rotación sensor -> mundo.
int quaternion_rotate_vector(quaternion_t q, const float vector[3], float result[3]);

#endif