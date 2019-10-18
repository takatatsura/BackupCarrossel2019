#include "calculaCampoPotencial.h"
#include "Auxiliares.h"
#include <queue>

using namespace std;

queue<PosXY> pilha;
extern int mapa[170 / 2][130 / 2];
extern int quadro;

void iniciaMapa() {
  for (int x = 0; x < 85; x++) {
    for (int y = 0; y < 65; y++) {
      mapa[x][y] = VAZIO;
    }
  }

  // Laterais do campo
  for (int x = 5; x < 85 - 5; x++) {
    mapa[x][0] = mapa[x][64] = OBSTACULO;
    mapa[x][1] = mapa[x][64 - 1] = OBSTACULO;
  }
  // Linha de fundo
  for (int y = 0; y <= 23; y++) {
    mapa[5][y] = mapa[84 - 5][y] = OBSTACULO;
    mapa[6][y] = mapa[84 - 6][y] = OBSTACULO;
  }
  // Fundo do gol
  for (int y = 22; y <= 22 + 20; y++) {
    mapa[0][y] = mapa[84][y] = OBSTACULO;
    mapa[1][y] = mapa[84 - 1][y] = OBSTACULO;
  }
  // Linha de fundo
  for (int y = 21 + 20; y <= 64; y++) {
    mapa[5][y] = mapa[84 - 5][y] = OBSTACULO;
    mapa[6][y] = mapa[84 - 6][y] = OBSTACULO;
  }
  // Laterais do gol
  for (int x = 2; x < 5; x++) {
    mapa[x][22] = mapa[84 - x][22] = OBSTACULO;
    mapa[x][23] = mapa[84 - x][23] = OBSTACULO;
    mapa[x][22 + 20] = mapa[84 - x][22 + 20] = OBSTACULO;
    mapa[x][21 + 20] = mapa[84 - x][21 + 20] = OBSTACULO;
  }
}

void colocaObjeto(int x, int y, int raio) {
  raio = raio / 2 + 1;
  int distQuadrado = raio * raio;
  x /= 2;
  y /= 2;
  for (int xi = x - raio; xi <= x + raio; xi++) {
    for (int yi = y - raio; yi <= y + raio; yi++) {
      if (xi >= 0 && xi < 85 && yi >= 0 && yi < 64 &&
          (x - xi) * (x - xi) + (y - yi) * (y - yi) <= distQuadrado) {
        mapa[xi][yi] = OBSTACULO;
      }
    }
  }
}

void colocaObjetos(Estado estado[], int numRoboDonoDoCampo, bool protegeBola,
                   bool ignoraAdversarios = true) {
  int numObjs = ignoraAdversarios ? NUM_ROBOS_TIME + 1 : NUM_ROBOS_TIME * 2 + 1;
  for (int i = 0; i < numObjs; i++) {
    if (i == NUM_ROBOS_TIME) { // se é a bola
      if (!protegeBola) {
        colocaObjeto(estado[i].x, estado[i].y, 5);
      }
    } else {
      if (numRoboDonoDoCampo != i)
        colocaObjeto(estado[i].x, estado[i].y, 7);
    }
  }
}

void criaCaminhoFinalNoAngulo(int objx, int objy, int objAng, int objVel) {
  int ox = objx / 2;
  int oy = objy / 2;
  int oa = objAng + 180;
  int dProtecao = objVel + 1; // Tamanho da proteção, depende da velocidade
  int dx = 0, dy = 0;

  if (dProtecao > 5) {
    dProtecao = 5;
  }

  int dCurva = dProtecao;

  // Limpa a pilha do menor caminho
  while (!pilha.empty()) { // Limpa pilha
    pilha.pop();
  }

  // Verifica se existe obstáculo no caminho que será criado; se tem, diminui a
  // velocidade e o bloquei ao redor do caminho
  float fdx = coss(oa);
  float fdy = seno(oa);
  for (int i = 1; i <= dProtecao; i++) {
    dx = round(fdx * i);
    dy = round(fdy * i);
    int x = ox + dx;
    int y = oy + dy;
    if (x < 1 || x > 84 || y < 1 || y > 64 || mapa[x][y] == OBSTACULO) {
      dCurva = i - 2;
      break;
    }
  }

  int dMax = dCurva * dCurva;

  for (int dx = -dCurva; dx <= dCurva; dx++) {
    for (int dy = -dCurva; dy <= dCurva; dy++) {
      int ang = atang2(dy, dx);
      int dAng = abs(difAngMenos180a180(ang, oa));
      if (dAng > 15) {
        if (dx * dx + dy * dy < dMax) {
          int x = ox + dx;
          int y = oy + dy;
          if (x > 0 && x < 84 && y > 0 && y < 64) {
            mapa[x][y] = OBSTACULO;
          }
        }
      }
    }
  }
  if (ox > 0 && ox < 84 && oy > 0 && oy < 64) {
    mapa[ox][oy] = 0;
    pilha.push(PosXY(ox, oy));
  }
  dx = round(fdx);
  dy = round(fdy);
  int x = ox + dx;
  int y = oy + dy;
  if (x > 0 && x < 84 && y > 0 && y < 64) {
    mapa[x][y] = 0;
    pilha.push(PosXY(x, y));
  }
}

void criaCampo() {
  while (!pilha.empty()) {
    PosXY pto = pilha.front();
    int x = pto.x;
    int y = pto.y;
    pilha.pop();
    int pot;
    for (int xi = x - 1; xi <= x + 1; xi++) {
      for (int yi = y - 1; yi <= y + 1; yi++) {
        if (xi == x || yi == y) {
          pot = mapa[x][y] + 10;
        } else {
          pot = mapa[x][y] + 14; // na diagonal 10 * sqrt(2)
        }
        if (mapa[xi][yi] < OBSTACULO && mapa[xi][yi] > pot) {
          mapa[xi][yi] = pot;
          pilha.push(PosXY(xi, yi));
        }
      }
    }
  }
}

int direcao(int numRobo, int xAtual, int yAtual, int objx, int objy, int objAng,
            int objVel) {
  int x = xAtual / 2;
  int y = yAtual / 2;
  if (mapa[x][y] == VAZIO) {
    return atang2(objy / 2 - y, objx / 2 - x);
  } else {
    int menor = mapa[x][y];
    int xMenor = x, yMenor = y;
    for (int i = -2; i <= 2; i++) {
      int d = abs(i) + 2; // abs(i)+2 é distancia Manhattan
      d *= 4.5;
      if (mapa[x + i][y - 2] + d < menor) {
        menor = mapa[x + i][y - 2] + d;
        xMenor = x + i;
        yMenor = y - 2;
      }
      if (mapa[x + i][y + 2] + d < menor) {
        menor = mapa[x + i][y + 2] + d;
        xMenor = x + i;
        yMenor = y + 2;
      }
      if (mapa[x - 2][y + i] + d < menor) {
        menor = mapa[x - 2][y + i] + d;
        xMenor = x - 2;
        yMenor = y + i;
      }
      if (mapa[x + 2][y + i] + d < menor) {
        menor = mapa[x + 2][y + i] + d;
        xMenor = x + 2;
        yMenor = y + i;
      }
    }
    if (yMenor - y == 0 && xMenor - x == 0) {
      return ANG_INVALIDO;
    } else {
      return atang2(yMenor - y, xMenor - x);
    }
  }
}

void calculaCampoPotencial(int numRobo, int objx, int objy, int objAng,
                           int objVel, bool protegeBola, bool ignoraAdversario,
                           Estado estado[]) {
  // if (numRobo == 0) {
  iniciaMapa();
  colocaObjetos(estado, numRobo, protegeBola);
  criaCaminhoFinalNoAngulo(objx, objy, objAng, objVel);
  criaCampo();
  // }
}