/*
 * Single_Cell.h
 *
 *  Created on: Mar 8, 2024
 *      Author: namontoy
 *
 * Este archivo esta pensado describir a una casilla (celda) del mapa,
 * en un intento de pensar a C y sus estructuras como los objetos de la
 * OOP.
 */

#ifndef SINGLE_CELL_H_
#define SINGLE_CELL_H_

#include <stdint.h>

/* Esta estructura "describe" todos los elementos y caracteristicas de una celda X,Y
 * del mapa, como por ejemplo:
 * - Identificación
 * - ubicación
 * - Que tipo de celda es
 * - Valores G, H y F
 * - Lista de sus vecinos
 * - Indicador de quien es su parent.
 * */
typedef struct
{
    uint8_t id;
    uint8_t pos_x;
    uint8_t pos_y;
    uint8_t type;
    uint8_t g;
    uint8_t h;
    uint8_t f;
    uint8_t parent;
    uint8_t neighbors[8];
    uint8_t state;
} Cell_map_t;

enum type {
    FREE = 0,
    OBSTACLE,
    START,
    GOAL
};

enum state{
    NONE = 0,
    OPEN,
    CLOSE,
    
};


/** Prototipos publicos */
Cell_map_t create_cell(uint8_t pos_x, uint8_t pos_y);

#endif /* SINGLE_CELL_H_ */
