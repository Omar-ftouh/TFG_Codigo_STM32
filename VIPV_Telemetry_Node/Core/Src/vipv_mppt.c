/*
 * vipv_mppt.c
 *
 *  Created on: May 31, 2026
 *      Author: omarf
 */

#include "vipv_mppt.h"

// ----------------- EQUIVALENCIA A LIBRERÍA numpy DE PYTHON --------------------

// 1. Función para calcular la potencia simulada (Interpolación Lineal)
float obtener_potencia_panel(float v_actual, const float *v_datos, const float *i_datos, uint8_t tamano_array) {

    // Clipping: Evitar salir de los límites del array (Equivalente a np.clip en Python)

	// Si el voltaje pedido es menor que el mínimo del array, se limita al mínimo
    if (v_actual <= v_datos[0]) {
        return v_actual * i_datos[0];
    }
    // Si el voltaje pedido es mayor que el máximo, se limita al máximo
    if (v_actual >= v_datos[tamano_array - 1]) {
        return v_actual * i_datos[tamano_array - 1];
    }


    // Interpolación lineal (Equivalente a np.interp en Python)

    // Búsqueda de entre qué dos puntos exactos del array nos encontramos
    for (uint8_t i = 0; i < tamano_array - 1; i++) {
        if (v_actual >= v_datos[i] && v_actual <= v_datos[i + 1]) {

        	//Fórmula matemática de la ecuación de la recta entre 2 puntos:
            float x0 = v_datos[i];
            float y0 = i_datos[i];
            float x1 = v_datos[i + 1];
            float y1 = i_datos[i + 1];

            float i_interpolada = y0 + ((v_actual - x0) * (y1 - y0)) / (x1 - x0);

            // Retornar potencia (P = V * I)
            return v_actual * i_interpolada;
        }
    }
    return 0.0f;
}



// 2. Algoritmo MPPT (Perturbar y Observar)
float mppt_po(float v_actual, float p_actual, float v_anterior, float p_anterior, float paso_v) {

    float delta_p = p_actual - p_anterior;
    float delta_v = v_actual - v_anterior;
    float v_siguiente = v_actual;

    	// Si la potencia ha aumentado
        if (delta_p > 0) {
            if (delta_v > 0) {
                v_siguiente = v_actual + paso_v;  // Se mantiene el sentido (+)
            } else {
                v_siguiente = v_actual - paso_v;  // Se mantiene el sentido (-)
            }
        }
        // Si la potencia ha disminuido
        else {
            if (delta_v > 0) {
                v_siguiente = v_actual - paso_v;  // Se invierte el sentido (-)
            } else {
                v_siguiente = v_actual + paso_v;  // Se invierte el sentido (+)
            }
        }

    return v_siguiente;
}
