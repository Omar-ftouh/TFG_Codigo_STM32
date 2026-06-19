/*
 * vipv_mppt.h
 *
 *  Created on: May 31, 2026
 *      Author: omarf
 */

#ifndef INC_VIPV_MPPT_H_
#define INC_VIPV_MPPT_H_

#include <stdint.h>
#include <math.h>

float obtener_potencia_panel(float v_actual, const float *v_datos, const float *i_datos, uint8_t tamano_array);
float obtener_potencia_directa(float v_actual, const float *array_v, float *array_p, int tamano);
float mppt_po(float v_actual, float p_actual, float v_anterior, float p_anterior, float paso_v);
float obtener_potencia_dinamica(float v_actual, float irr_actual);
float ejecutar_mppt_adaptativo(float v_actual, float irr_actual, float v_anterior, float p_anterior, float paso_v, float p_actual);


// DECLARACIONES EXTERNAS CORRECTAS
extern const float v_vector[50];
extern float p_sol[50];
extern float p_sombra[50];
extern const float IRR_MAX;
extern const float IRR_MIN;


// VARIABLES PARA EL MPPT ADAPTATIVO
#define UMBRAL_BRUSCO       150.0f  // Variación de irradiancia (W/m2) considerada como fluctuación brusca de sombra/Sol
#define LIMITE_CAMBIOS      3       // Cuántos cambios rápidos se toleran antes de congelar el P&O
#define TIEMPO_CONGELACION  5       // Segundos (iteraciones del bucle) que el MPPT se quedará fijo
#define VOLTAJE_PUNTO_FIJO  19.5f   // Punto fijo a seguir al congelar el P&O (cercano al punto de máxima potencia promedio)


#endif /* INC_VIPV_MPPT_H_ */
