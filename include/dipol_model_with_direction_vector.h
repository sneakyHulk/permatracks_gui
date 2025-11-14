#pragma once
void dipol_model_with_direction_vector(double *out, double H1, double R1, double Br1, double x1, double y1, double z1, double mx1, double my1, double mz1, double Gx, double Gy, double Gz);
void dipol_model_with_direction_vector_jacobian_position(double *out, double H1, double R1, double Br1, double x1, double y1, double z1, double mx1, double my1, double mz1);
void dipol_model_with_direction_vector_jacobian_direction(double *out, double H1, double R1, double Br1, double x1, double y1, double z1);
void dipol_model_with_direction_vector_jacobian_offset(double *out);
