#ifndef CALCULACAMPOPOTENCIAL_H
#define CALCULACAMPOPOTENCIAL_H

#include "TiposClasses.h"

#define VAZIO 10000
#define OBSTACULO 10001
#define ANG_INVALIDO 10002

void iniciaMapa();

int direcao(int numRobo, int xAtual, int yAtual, int objx, int objy, int objAng,
            int objVel);
void calculaCampoPotencial(int numRobo, int objx, int objy, int objAng,
                           int objVel, bool protegeBola, bool ignoraAdversario,
                           Estado estado[]) ;

#endif /* CALCULACAMPOPOTENCIAL_H */