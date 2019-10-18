/*
 * ESTE software foi fornecido como exemplo de controlador de futebol de robôs
 na Segunda Oficina Brasileira de Futebol de Robôs realizada junto ao 5o
 Workshop em Automação e Robótica Aplicada (Robocontrol) 2010.

 * Você que está de posse dESTE software, está livre para utilizá-lo, alterá-lo,
 copiá-lo e incluí-lo parcial ou integralmente em outros software desde que
 acompanhado da seguinte indicação:
 * "Este software tem seções de código desenvolvidas por Rene Pegoraro no
 Laboratório de Integração de Sistemas e Dispositivos Inteligentes (LISDI) do
 Departamento de Computação da Faculdade de Ciências da Universidade Estadual
 Paulista (UNESP) - Campos de Bauru - SP - Brasil"

 * Se qualquer publicação for gerada pela utilização de software utilizando
 parcial ou integralmente ESTE software, esta publicação deve conter os devidos
 créditos para o "Grupo de Integração de Sistemas e Dispositivos Inteligentes
 (GISDI) do Departamento de Computação da Faculdade de Ciências da Universidade
 Estadual Paulista (UNESP) - Campos de Bauru - SP - Brasil"
 */

#include "Auxiliares.h"
#include "Controle.h"
#include "TiposClasses.h"
#include "calculaCampoPotencial.h"
//#include "Serial.h"



enum EstadoDoJogo {
  ATAQUE_A, // Modo de Ataque Caso A
  ATAQUE_B, // Modo de Ataque Caso B
  DEFESA_A, // Entrou no modo de defesa antes de 60% do campo ou entrou em DEFESA_B mas a bola chegou a 40% do campo
  DEFESA_B, // Entrou no modo de defesa apos 60% do campo
  TIRODEMETA,
  PENALIDADE
} estadoDoJogo;

bool colisao(int numObj1, int numObj2);

/** Matriz usada para o campó potencial
 */
extern int mapa[170 / 2][130 / 2];

extern int maxPixelsPorSegundo; // velocidade maxima que um robo pode alcancar
extern Estado estado[NUM_ROBOS_TIME * 2 + 1], estadoAnt[NUM_ROBOS_TIME * 2 + 1],
    estadoPrev[NUM_ROBOS_TIME * 2 + 1];
Estado estadoPrevVolante[NUM_ROBOS_TIME * 2 + 1],
    estadoPrevAtacante[NUM_ROBOS_TIME * 2 + 1];
extern Lado nossoLado;
extern bool emJogo;
extern bool tiroMeta;
extern bool emPenalidade;
extern bool emPosiciona;
extern bool emInicio;

extern Objetivo objetivoRobo[NUM_ROBOS_TIME];

enum EstadoAtacante { POSICIONA, ATACA } estadoAtacante;
int trocarVolanteAtacante = 0;
int contador = 0; // Serve para controlar a troca de estados do atacante

CmdEnviado cmdEnviado[TAM_CMD_ENV][NUM_ROBOS_TIME]; // comando enviado aos robos

extern int indGoleiro;
extern int indVolante;
extern int indAtacante;

void IniciaHistoriaCmdEnviado(void) {
  int i, j;
  for (i = 0; i < NUM_ROBOS_TIME; i++) {
    for (j = 0; j < TAM_CMD_ENV; j++) {
      cmdEnviado[j][i].dir = 0;
      cmdEnviado[j][i].esq = 0;
    }
  }
}

void SAI(char *s) {
  FILE *fp = fopen("debug.txt", "at");
  fprintf(fp, s);
  fflush(fp);
  fclose(fp);
}

void calculaPrevisao(int numRobo) {
  float x, y, dx, dy, ang, dAng, ve, vd;
  ang = estado[numRobo].angulo;
  x = estado[numRobo].x;
  y = estado[numRobo].y;

  for (int i = N_IMAGENS_INERCIA_ROBO + N_IMAGENS_ATRASO - 1; i >= 0; i--) {
    ve = cmdEnviado[i][numRobo].esq / FATOR_VEL_ROBO_UNID_POR_CM;
    vd = cmdEnviado[i][numRobo].dir / FATOR_VEL_ROBO_UNID_POR_CM;
    float v = (vd + ve) / 2;

    dAng = (vd - ve) / DIST_ENTRE_RODAS * 180 / M_PI;
    ang += dAng;

    dx = v * coss(ang);
    dy = v * seno(ang);

    x += dx;
    y += dy;

    if (x > TAM_X_CAMPO - TAM_X_DO_GOL) {
      x = TAM_X_CAMPO - TAM_X_DO_GOL;
    } else if (x < TAM_X_DO_GOL) {
      x = TAM_X_DO_GOL;
    }
    if (y > TAM_Y_CAMPO - TAM_ROBO) {
      y = TAM_Y_CAMPO - TAM_ROBO;
    } else if (y < TAM_ROBO) {
      y = TAM_ROBO;
    }
  }

  estadoPrev[numRobo].x = x;
  estadoPrev[numRobo].y = y;
  estadoPrev[numRobo].angulo = ang;
  estadoPrev[numRobo].dx = dx;
  estadoPrev[numRobo].dy = dy;
  estadoPrev[numRobo].dAngulo = dAng;

  // if (numRobo == 0) {
  //   FILE *fp = fopen("dados2.csv", "at");
  //   fprintf(fp, "%.0f, %.0f, %.0f, (%d %d), %.0f, %.0f, %.0f\n",
  //           estado[numRobo].x, estado[numRobo].y, estado[numRobo].angulo,
  //           cmdEnviado[0][0].esq, cmdEnviado[0][0].dir,
  //           estadoPrev[numRobo].x, estadoPrev[numRobo].y,
  //           estadoPrev[numRobo].angulo);
  //   fclose(fp);
  // }

  // if (quadro >= 34) {
  //   cout << "debug" << endl;
  // }
}

void calculaPrevisao(void) {
#ifdef SEM_PREVISAO
  for (int i = 0; i < 7; i++) {
    estadoPrev[i] = estado[i];
  }
#else
  for (int i = 0; i < NUM_ROBOS_TIME; i++) {
    calculaPrevisao(i);
  }
#endif
  // Previsao da bola e dos robôs adversários
  for (int i = NUM_ROBOS_TIME; i < NUM_ROBOS_TIME * 2 + 1; i++) {
    float x, y;
    float dx, dy;
    // media do angulo
    dx = estado[i].dx * 0.75 + estadoAnt[i].dx * 0.25;
    dy = estado[i].dy * 0.75 + estadoAnt[i].dy * 0.25;
    estadoPrev[i].angulo = atang2(dy, dx) * 0.5 + estado[i].angulo * 0.3 +
                           estadoAnt[i].angulo * 0.2;
    estadoPrev[i].dx = dx;
    estadoPrev[i].dy = dy;

    // while
    x = estado[i].x + dx * N_IMAGENS_ATRASO;
    y = estado[i].y + dy * N_IMAGENS_ATRASO;
    if (x > TAM_X_CAMPO - TAM_X_DO_GOL) {
      x = TAM_X_CAMPO - TAM_X_DO_GOL;
    } else if (x < TAM_X_DO_GOL) {
      x = TAM_X_DO_GOL;
    }
    if (y > TAM_Y_CAMPO - TAM_ROBO) {
      y = TAM_Y_CAMPO - TAM_ROBO;
    } else if (y < TAM_ROBO) {
      y = TAM_ROBO;
    }
    estadoPrev[i].x = x;
    estadoPrev[i].y = y;
  }

  for (int i = 0; i < NUM_ROBOS_TIME * 2 + 1; i++) {
    for (int j = i + 1; j < NUM_ROBOS_TIME * 2 + 1; j++) {
      colisao(i, j);
    }
  }
}

// Retorna:	Numero de quadros ate o choque
//		Numero do elemento no caminho (-1 nenhum)
void calculaChoque(int numRobo, bool consideraBola, float *xObjetivo,
                   float *yObjetivo, float deslocXObjetivo,
                   float deslocYObjetivo, int *NQuadros, int *NumElemChoque) {
  float xRobo, yRobo, dxRobo, dyRobo;
  float deslocXObjetivoRobo, deslocYObjetivoRobo, deslocXAbs, deslocYAbs;
  int N, nx, ny;

  xRobo = estadoPrev[numRobo].x;
  yRobo = estadoPrev[numRobo].y;
  dxRobo = estadoPrev[numRobo].dx;
  dyRobo = estadoPrev[numRobo].dy;

  deslocXAbs = abs(deslocXObjetivoRobo =
                       *xObjetivo - xRobo); // Distancia entre objetivo e robo
  deslocYAbs = abs(deslocYObjetivoRobo = *yObjetivo - yRobo);

  float ddxObjRobo = deslocXObjetivo - dxRobo;
  float ddyObjRobo = deslocYObjetivo - dyRobo;

  if (ddxObjRobo != 0) {
    nx = deslocXObjetivoRobo / ddxObjRobo;
  } else
    nx = 30;
  if (ddyObjRobo != 0) {
    ny = deslocYObjetivoRobo / ddyObjRobo;
  } else
    ny = 30;

  N = max(nx, ny);
  if (N < 0 || N > 30) { // Se mais que 30 quadros (1s), desconsidera.
    N = 30;              // Limita em um segundo
  }
  *NQuadros = N;

  if (deslocYAbs + deslocXAbs <=
      TAM_ROBO) { // Se distancia e' menor que TAM_ROBO, nao cabe elem. entre
                  // robo e objetivo
    *NumElemChoque = -1; // Nao ocorrera choque
  } else {
    float a = deslocYObjetivoRobo /
              deslocXObjetivoRobo; // reta entre o robo e o objetivo
    float b = yRobo - a * xRobo;
    for (int i = 0; i < NUM_ROBOS_TIME * 2 + 1; i++) {
      if (i != numRobo && (consideraBola || i != IND_BOLA)) {
        float xi, yi, dxi, dyi;
        xi = estadoPrev[i].x;
        yi = estadoPrev[i].y;
        dxi = estadoPrev[i].dx;
        dyi = estadoPrev[i].dy;
        int nAteObst = (yi - a * xi - b) / (a * dxi - dyi);
        if (nAteObst > 0) { // se nAteObst > 0, o encontro (choque) pode ocorrer
          if (nAteObst < *NQuadros) { // nAteObst < *NQuadros, o encontro
                                      // (choque) pode ocorrer (e vai retornar o
                                      // cheque mais proximo do Robo
            *NumElemChoque = i;
            *NQuadros = nAteObst;
            float xRoboChoque =
                xRobo +
                dxRobo * (nAteObst - 1); // -1 pois sabendo um pouco antes fica
                                         // melhor para desviar
            float yRoboChoque = yRobo + dyRobo * (nAteObst - 1);
            float xiChoque =
                xi + dxi * (nAteObst - 1); // -1 pois sabendo um pouco antes
                                           // fica melhor para desviar
            float yiChoque = yi + dyi * (nAteObst - 1);
            if (xRoboChoque > xiChoque) {
              xRoboChoque += TAM_ROBO;
              if (xRoboChoque > TAM_X_CAMPO - TAM_X_DO_GOL - TAM_ROBO)
                xRoboChoque -= TAM_ROBO * 3; // se nao da para passar por um
                                             // lado, desvia bastante pelo outro
            } else {
              xRoboChoque -= TAM_ROBO;
              if (xRoboChoque < TAM_ROBO)
                xRoboChoque += TAM_ROBO * 3;
            }
            if (yRoboChoque > yiChoque) {
              if (yRoboChoque > TAM_Y_CAMPO - TAM_ROBO)
                yRoboChoque -= TAM_ROBO * 3;
              yRoboChoque += TAM_ROBO;
            } else {
              yRoboChoque -= TAM_ROBO;
              if (yRoboChoque < TAM_ROBO)
                yRoboChoque += TAM_ROBO * 3;
            }

            *xObjetivo = yRoboChoque;
            *yObjetivo = yRoboChoque;
          }
        }
      }
    }
  }
}

bool existeAdversarioProximo(float x, float y, float dist) {
  float quadDist = dist * dist;
  for (int i = IND_BOLA + 1; i < NUM_ROBOS_TIME * 2 + 1; i++) {
    float dx = x - estadoPrev[i].x;
    float dobroDx = dx * dx;
    float dy = y - estadoPrev[i].y;
    float dobroDy = dy * dy;
    if (dobroDx + dobroDy < quadDist) {
      return true;
    }
  }
  return false;
}

bool isRoboEmPosseBola(int indiceRobo){
  float sentidoRoboX, sentidoRoboY, roboBolaX, roboBolaY;
  sentidoRoboX = cos(estadoPrev[indiceRobo].angulo*M_PI/180 - M_PI);
  sentidoRoboY = sin(estadoPrev[indiceRobo].angulo*M_PI/180 - M_PI);
  roboBolaX = estadoPrev[IND_BOLA].x - estadoPrev[indiceRobo].x;
  roboBolaY = estadoPrev[IND_BOLA].y - estadoPrev[indiceRobo].y;
  /*
         /  45 graus em relacao ao sentido do robo
   _____/
  |     |
  |     |   A bola deve estar entre dois vetores a 45 graus e -45 graus 
  |_____|   em relacao ao sentido do robo
         \
          \ 45 graus em relacao ao sentido do robo
*/

  //Formula de encontrar angulo entre dois vetores
	//Resultado deve ser menor do que PI/4 (0.785398163397448309616) para a posse ser considerada
  return ((std::abs(std::acos(( (sentidoRoboX * roboBolaX + sentidoRoboY * roboBolaY) / (sqrt(pow(sentidoRoboX, 2) + 
          pow(sentidoRoboY, 2)) * sqrt(pow(roboBolaX, 2) + pow(roboBolaY, 2)) ) ))) < 0.785398163397448309616)
			&& (sqrt((roboBolaX * roboBolaX) + (roboBolaY * roboBolaY)) <= TAM_ROBO*1.5)); // Um robo inteiro era muito grande
}

//==================== G O L E I R O =========================
void EstrGoleiro(void) {
  float xObjetivo, yObjetivo, xGoleiro, yGoleiro, dxGoleiro, dyGoleiro, xBola,
      yBola, dxBola, dyBola, N, yBolaQuandoChegarNoGol;
  xGoleiro = estadoPrev[indGoleiro].x;
  yGoleiro = estadoPrev[indGoleiro].y;
  dxGoleiro = estadoPrev[indGoleiro].dx; // 
  dyGoleiro = estadoPrev[indGoleiro].dy; //
  xBola = estadoPrev[IND_BOLA].x;
  yBola = estadoPrev[IND_BOLA].y;
  dxBola = estadoPrev[IND_BOLA].dx; // Se negativo a bola esta indo para o nosso gol, se positivo, o contrario
  dyBola = estadoPrev[IND_BOLA].dy; // Se negativo a bola esta descendo, o oposto eh verdadeiro

  xObjetivo = POS_X_GOLEIRO;
  yObjetivo = yGoleiro;

  switch (estadoDoJogo) {
    case TIRODEMETA:
        xObjetivo = CENTRO_X_GOL;
        yObjetivo = CENTRO_Y_GOL;
        if (xGoleiro > TAM_X_DO_GOL * 3)
          tiroMeta = false;
        break;
    
    case PENALIDADE:
        xObjetivo = TAM_X_DO_GOL;
        if (dxBola > 2) {
          emPenalidade = false;
        }
        break;  
  }

  if (xBola < TAM_X_CAMPO * 0.5500) { // Bola antes da metade do campo
    if (abs(dxBola) > 1) {            // Bola em movimento
      N = (POS_X_GOLEIRO - xBola) / dxBola;
      if (N > 0) {
        yBolaQuandoChegarNoGol =
            N * dyBola + yBola; // Calcula y da bola quando ela chegar no gol
        if (yBolaQuandoChegarNoGol <
            (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 -
                TAM_ROBO) { // Bola indo para linha de fundo
          yObjetivo = (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 - TAM_ROBO;
        } else if (yBolaQuandoChegarNoGol >
                    TAM_Y_CAMPO - (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 +
                        TAM_ROBO) {
          yObjetivo =
              TAM_Y_CAMPO - (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 + TAM_ROBO;
        } else {
          yObjetivo = yBolaQuandoChegarNoGol; // Posiciona o robo alinhado com
                                              // a chegada da bola
        }
      } else {                            // a bola esta se afastando do gol
        if (xBola > TAM_X_CAMPO * 0.25) { // Bola 80-10cm do gol
          yObjetivo = yGoleiro;
        } else {
          if (yBola < TAM_Y_CAMPO/2 - TAM_Y_DO_GOL/2 - TAM_ROBO) {
            yObjetivo = TAM_Y_CAMPO/2 - TAM_Y_DO_GOL/2 - TAM_ROBO;
          } else if (yBola > TAM_Y_CAMPO/2 + TAM_Y_DO_GOL/2 +
                                  TAM_ROBO) {
            yObjetivo = TAM_Y_CAMPO/2 + TAM_Y_DO_GOL/2 + TAM_ROBO;
          } else {
            yObjetivo = yBola; // Robo acompanha a posicao y da bola
          }
        }
      }
    } else { // bola parada
      if (xBola > TAM_X_CAMPO * 0.25) {
        yObjetivo = TAM_Y_CAMPO / 2; // Robo no centro do gol
      } else {
        if (yBola < (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 - TAM_ROBO * 2) {
          yObjetivo = (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 - TAM_ROBO * 2;
        } else if (yBola > TAM_Y_CAMPO - (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 +
                                TAM_ROBO * 2) {
          yObjetivo =
              TAM_Y_CAMPO - (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 + TAM_ROBO * 2;
        } else {
          yObjetivo = yBola; // Robo acompanha a posicao y da bola
        }
      }
    }
  } else {
    yObjetivo = TAM_Y_CAMPO / 2;
  }

  //	Se o goleiro está erroscado na parede ao redor do gol, movimentoa o
  // robô. 	if (xGoleiro < POS_X_GOLEIRO + d && (yGoleiro < (TAM_Y_CAMPO -
  // TAM_Y_DO_GOL) / 2 || yGoleiro > TAM_Y_CAMPO
  //			- (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2)) {// Fora do gol
  //		if (abs(dxGoleiro) <= 1 && abs(dyGoleiro) <= 1) { // Esta parado
  //			xObjetivo = POS_X_GOLEIRO + d; // Afasta robo da parede
  //		}
  //	}

  objetivoRobo[indGoleiro].x = xObjetivo;
  objetivoRobo[indGoleiro].y = yObjetivo;
  objetivoRobo[indGoleiro].angulo = 90; // graus
  objetivoRobo[indGoleiro].vel = 0;
}

//==================== V O L A N T E =========================
void estrVolante(int indVolante) {
  int xObjetivo, yObjetivo, angRobo, xRobo, yRobo, dxRobo, dyRobo, xBola, yBola,
      dxBola, dyBola, xAtacante, yAtacante, velObjetivo, angObjetivo;
  float yAlinhamentoRoboBolaGol;
  angRobo = estadoPrev[indVolante].angulo;
  xRobo = estadoPrev[indVolante].x;
  yRobo = estadoPrev[indVolante].y;
  dxRobo = estadoPrev[indVolante].dx;
  dyRobo = estadoPrev[indVolante].dy;
  xBola = estadoPrev[IND_BOLA].x;
  yBola = estadoPrev[IND_BOLA].y;
  dxBola = estadoPrev[IND_BOLA].dx;
  dyBola = estadoPrev[IND_BOLA].dy;
  xAtacante = estadoPrev[indAtacante].x;
  yAtacante = estadoPrev[indAtacante].y;

  velObjetivo = 0;
  angObjetivo = 90;
  xObjetivo = xRobo; // Por padrao mantem o robo na linha atual
  yObjetivo = yBola; // Por padrao fica alinhado com a bola
  
  switch (estadoDoJogo){
    case ATAQUE_A:
        xObjetivo = xAtacante + 2*TAM_ROBO;
        if(yAtacante <= TAM_Y_CAMPO/2)
          yObjetivo = yAtacante + 1.5*TAM_ROBO;
        else
          yObjetivo = yAtacante - 1.5*TAM_ROBO;

        // Se o objetivo do volante for muito perto do gol adversario, recua um pouco para nao entrar no gol
        if(xObjetivo > 0.9 * TAM_X_CAMPO){
          xObjetivo = 0.9 * TAM_X_CAMPO;
        }
        break;

    case ATAQUE_B:
        xObjetivo = xAtacante - 20;
        yObjetivo = yAtacante;
        break;

    case DEFESA_A:
        // Fica circulando no perimetro da area do gol

        // A bola esta a frente da area em x
        if (xBola > TAM_X_DO_GOL + TAM_X_AREA){
          xObjetivo = TAM_X_DO_GOL + TAM_X_AREA + TAM_ROBO/2;
          // Se a bola esta com o y acima da area
          if (yBola > TAM_Y_CAMPO/2 + TAM_Y_AREA/2)
            yObjetivo = TAM_Y_CAMPO/2 + TAM_Y_AREA/2 + TAM_ROBO/2;
          // Se a bola esta com o y abaixo da area
          else if (yBola < TAM_Y_CAMPO/2 - TAM_Y_AREA/2)
            yObjetivo = TAM_Y_CAMPO/2 - TAM_Y_AREA/2 - TAM_ROBO/2;
          // A bola esta a frente da area
          else 
            yObjetivo = yBola;
        }
        // A bola esta acima da area
        else if (yBola > TAM_Y_CAMPO/2 + TAM_Y_AREA/2) {
          xObjetivo = xBola;
          yObjetivo = TAM_Y_CAMPO/2 + TAM_Y_AREA/2 + TAM_ROBO/2;
        }
        // A bola esta abaixo da area
        else if (yBola < TAM_Y_CAMPO/2 - TAM_Y_AREA/2){
          xObjetivo = xBola;
          yObjetivo = TAM_Y_CAMPO/2 - TAM_Y_AREA/2 - TAM_ROBO/2;
        }
        // A bola esta dentro da area
        else{
          // O robo nao fica exatamente na frente da bola
          // pois isso pode atrapalhar
          xObjetivo = TAM_X_DO_GOL + TAM_X_AREA + TAM_ROBO/2;
          yObjetivo = yBola + TAM_ROBO*1.5;
        }

        // Se, estando no modo DEFESA_A a bola passar de 40% do campo e estiver indo para o gol adversario
        // o volante avança na bola
        if(dxBola > 0 && xBola > 0.4 * TAM_X_CAMPO){
          xObjetivo = xBola;
          yObjetivo = yBola;
        }
        break;

    case DEFESA_B:
        xObjetivo = xBola;
        yObjetivo = yBola;
        angObjetivo = 0;
  }

  float dx = xRobo - xAtacante;
  dx *= dx;
  float dy = yRobo - yAtacante;
  dy *= dy;
  float quadDist = TAM_ROBO * 2;
  quadDist *= quadDist;

  if (dx + dy < quadDist) {  // Se robos muito proximos
    if (yAtacante > yRobo) { // Repele em Y
      yObjetivo = yAtacante - TAM_ROBO * 2;
    } else {
      yObjetivo = yAtacante + TAM_ROBO * 2;
    }
    if (xAtacante > xRobo) { // Repele em X
      xObjetivo = xAtacante - TAM_ROBO * 2;
    } else {
      xObjetivo = xAtacante + TAM_ROBO * 2;
    }
  }

  if (xRobo > TAM_X_CAMPO - TAM_X_DO_GOL * 1.5) {
    xObjetivo = TAM_X_CAMPO * 0.8;
    yObjetivo = TAM_Y_CAMPO / 2;
  }

  objetivoRobo[indVolante].x = xObjetivo;
  objetivoRobo[indVolante].y = yObjetivo;
  objetivoRobo[indVolante].angulo = angObjetivo;
  objetivoRobo[indVolante].vel = velObjetivo;
  if(isRoboEmPosseBola(indVolante))
    trocarVolanteAtacante = true;
}

//==================== A T A C A N T E =========================
void estrAtacante(void) {
  float xObjetivo, yObjetivo, xRobo, yRobo, dxRobo, dyRobo, xBola, yBola,
      dxBola, dyBola, velObjetivo, angObjetivo;
  float dxBG, dyBG, dxBO, dyBO, m;
  float yAlinhamentoRoboBolaGol;

  xBola = estadoPrev[IND_BOLA].x;
  yBola = estadoPrev[IND_BOLA].y;
  dxBola = estadoPrev[IND_BOLA].dx;
  dyBola = estadoPrev[IND_BOLA].dy;

  xRobo = estadoPrev[indAtacante].x;
  yRobo = estadoPrev[indAtacante].y;
  dxRobo = estadoPrev[indAtacante].dx;
  dyRobo = estadoPrev[indAtacante].dy;

  dxBG = xBola -
         CENTRO_X_GOL; // Calcula objetivo atras bola, alinhado direcao Bola-Gol
  dyBG = yBola - CENTRO_Y_GOL;

  // Calcula o deslocamento de limite dependendo do angulo
  float d = abs((((int)estadoPrev[indVolante].angulo + 45) % 90) - 45) * 2 /
            45; // nos angulos 45, 135, 225 e 315 considera o robo maior 2 cm
  // a linha acima serve para dar mais espaco ao robo quando girando

  if (xBola - xRobo >
      0) { // Com o alinhamento robo-bola, calcula Y da linha do gol
    yAlinhamentoRoboBolaGol =
        yBola - (yBola - yRobo) * (xBola - CENTRO_X_GOL) / (xBola - xRobo);
  } else {
    yAlinhamentoRoboBolaGol = yBola;
  }

  if (xBola >= xRobo && // a bola esta a frente do robo
      xBola - xRobo < TAM_ROBO &&
      abs(yBola - yRobo) <
          TAM_ROBO / 2) {     // o robo esta proximo e alinhado com a bola
    if(estadoAtacante == POSICIONA && contador < 5){
      contador++;
    }
    else {
      yObjetivo = CENTRO_Y_GOL; // Vai para o gol, a bola deve estar no meio
      xObjetivo = CENTRO_X_GOL;
      angObjetivo = atang2(-dyBG, -dxBG);
      velObjetivo = 7;
      contador = 0;
      estadoAtacante = ATACA;
    }
  } else if (xBola >= xRobo && // a bola esta a frente do robo
             yAlinhamentoRoboBolaGol > (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 + 5 &&
             yAlinhamentoRoboBolaGol <
                 TAM_Y_CAMPO - (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 -
                     5) { // Se existe alinhamento robo-bola-gol
    // o +/- 5cm serve como faixa de histerese para evitar transicoes constantes
    // entre POSICIONA e ATACA) (CENTRO_Y_GOL - yAlinhamentoRoboBolaGol) é o
    // erro, colocando duas vezes o erro forcará o robo a melhor se alinhar
    if(estadoAtacante == POSICIONA && contador < 5){
      contador++;
    } else {
      yObjetivo = CENTRO_Y_GOL; // - (CENTRO_Y_GOL - yAlinhamentoRoboBolaGol) * 2;
                                // // Vai para o gol, a bola deve estar no meio
      xObjetivo = CENTRO_X_GOL;
      angObjetivo = atang2(-dyBG, -dxBG);
      velObjetivo = 7;
      contador = 0;
      estadoAtacante = ATACA;
    }
  } else {
    // o +/- 10cm serve como faixa de histerese para evitar transicoes
    // constantes entre ATACA e POSICIONA)
    if (!(abs(yRobo - yBola) <= TAM_ROBO && xBola - xRobo < TAM_ROBO * 5 &&
          estadoAtacante != ATACA)) {
      if (contador < 5){
        contador++;
      } else {
        /*if (xBola < xRobo || yAlinhamentoRoboBolaGol < (TAM_Y_CAMPO
        - TAM_Y_DO_GOL) / 2 - 10 || yAlinhamentoRoboBolaGol
        > TAM_Y_CAMPO - (TAM_Y_CAMPO - TAM_Y_DO_GOL) / 2 + 10) { //Nao existe
        mais alinhamento robo-bola-gol*/
        estadoAtacante = POSICIONA;
        contador = 0;
      }
    }
  }

  // ----------------------  estadoAtacante==POSICIONA
  // ---------------------------
  if (estadoAtacante == POSICIONA) { // Posicionando o Robo
    int N, R, dx, dy;
    static bool voltando = false;

    if (xRobo > xBola) {
      voltando = true;
    } else if (xRobo < xBola - TAM_ROBO * 2) {
      voltando = false;
    }

    if (voltando) {
      xObjetivo = xBola - TAM_ROBO * 2.5; // para sair do voltando
      if (yBola < yRobo) { // escolhe o lado para voltar atras da bola
        yObjetivo = yBola + TAM_ROBO * 2;
        angObjetivo = 240;
        if (yObjetivo > TAM_Y_CAMPO - TAM_ROBO) {
          yObjetivo = yBola - TAM_ROBO * 2;
        }
      } else {
        yObjetivo = yBola - TAM_ROBO * 2;
        angObjetivo = 120;
        if (yObjetivo < TAM_ROBO * 2) {
          yObjetivo = yBola + TAM_ROBO * 2;
        }
      }
      velObjetivo = 7;
    } else {
      m = max(abs(dxBG), abs(dyBG));
      if (xBola < xRobo) {
        dxBO =
            dxBG * TAM_ROBO * 2 / m; // 15cm atras da bola, posicao para chute
        dyBO = dyBG * TAM_ROBO * 2 / m;
      } else {
        dxBO =
            dxBG * TAM_ROBO * 2 / m; // 15cm atras da bola, posicao para chute
        dyBO = dyBG * TAM_ROBO * 2 / m;
      }
      xObjetivo = xBola + dxBO;
      yObjetivo = yBola + dyBO;
      velObjetivo = 3;
    }

    calculaChoque(indAtacante, false, &xObjetivo, &yObjetivo, dxBola, dyBola,
                  &N, &R);

    if (!voltando) {
      dx = xBola - xObjetivo; // Com a bola para quando desviar ja posicionar p/
                              // direcao bola-gol
      dy = yBola - yObjetivo;
      angObjetivo = atang2(dy, dx);
    }
    // ----------------------  estadoAtacante==ATACA ---------------------------
  } else {
    if (xBola - xRobo < TAM_ROBO &&
        abs(yBola - yRobo) <
            TAM_ROBO / 2) {     // o robo esta proximo e alinhado com a bola
      yObjetivo = CENTRO_Y_GOL; // Vai para o gol, a bola deve estar no meio
    } else {
      // (CENTRO_Y_GOL - yAlinhamentoRoboBolaGol) é o erro, colocando duas vezes
      // o erro forcará o robo a melhor se alinhar
      yObjetivo =
          CENTRO_Y_GOL; /* - (CENTRO_Y_GOL - yAlinhamentoRoboBolaGol)
                         * 2; // Vai para o gol, a bola deve estar no meio*/
    }
    xObjetivo = CENTRO_X_GOL;
    angObjetivo = atang2(-dyBG, -dxBG);
    velObjetivo = 7;
  }

  if (xObjetivo < TAM_X_DO_GOL + TAM_X_AREA +
                      TAM_ROBO / 2) { // Evita outro robo no gol defendendo
    if (xBola > xRobo && xBola > TAM_X_DO_GOL + TAM_X_AREA + TAM_ROBO / 2) {
      xObjetivo = xBola;
      angObjetivo = 0;
    } else {
      xObjetivo = TAM_X_DO_GOL + TAM_X_AREA + TAM_ROBO / 2;
      angObjetivo = 0;
    }
  } else if (xObjetivo > TAM_X_CAMPO - TAM_X_DO_GOL) {
    xObjetivo = TAM_X_CAMPO - TAM_X_DO_GOL;
    //				angObjetivo = 90;
  }

  // Serve para afastar o robo quando estiver muito perto das laterais
  if (yObjetivo < TAM_ROBO * 0.75 + d) {
    yObjetivo = TAM_ROBO * 0.75 + d;
    xObjetivo = xBola;
    angObjetivo = 0;
  } else if (yObjetivo > TAM_Y_CAMPO - TAM_ROBO * 0.75 - d) {
    yObjetivo = TAM_Y_CAMPO - TAM_ROBO * 0.75 - d;
    xObjetivo = xBola;
    angObjetivo = 0;
  }

  if (xBola < TAM_X_CAMPO / 2 &&
      xBola - xRobo < TAM_ROBO) { // tira bola do ataque
    xObjetivo = xBola;
    yObjetivo = yBola;
    angObjetivo = 0;
    velObjetivo = 7;
  } else if (xBola + yBola > TAM_X_CAMPO + TAM_Y_CAMPO -
                                 20) { // nao coloca robo no triangulo inutil
    xObjetivo = TAM_X_CAMPO - 20;
    yObjetivo = TAM_Y_CAMPO - TAM_ROBO / 2 - d;
    angObjetivo = 135;
    velObjetivo = 0;
  } else if (xBola - yBola > TAM_X_CAMPO - 20) {
    xObjetivo = TAM_X_CAMPO - 20;
    yObjetivo = TAM_ROBO + d;
    angObjetivo = 45;
    velObjetivo = 0;
  }
  /*

   float xGoleiro = estadoPrev[indGoleiro].x;
   float yGoleiro = estadoPrev[indGoleiro].y;
   float dx = xRobo - xGoleiro;
   dx *= dx;
   float dy = yRobo - yGoleiro;
   dy *= dy;
   float quadDist = TAM_ROBO * 2;
   quadDist *= quadDist;

   if (dx + dy < quadDist) { //Se robos muito proximos
   if (yGoleiro > yRobo) { //Repele em Y
   yObjetivo = yGoleiro - TAM_ROBO * 2;
   } else {
   yObjetivo = yGoleiro + TAM_ROBO * 2;
   }
   if (xGoleiro > xRobo) { //Repele em X
   xObjetivo = xGoleiro - TAM_ROBO * 2;
   } else {
   xObjetivo = xGoleiro + TAM_ROBO * 2;
   }
   }
   */

  if (xObjetivo <
      TAM_X_DO_GOL + TAM_X_AREA + TAM_ROBO) // se o robo está na área de defesa
    xObjetivo = TAM_X_DO_GOL + TAM_X_AREA + TAM_ROBO;

  objetivoRobo[indAtacante].x = xObjetivo;
  objetivoRobo[indAtacante].y = yObjetivo;
  objetivoRobo[indAtacante].angulo = angObjetivo;
  objetivoRobo[indAtacante].vel = velObjetivo;
}

void verificaPosiconamentos() {
  if (emPosiciona) {
    objetivoRobo[indGoleiro].x = 20;
    objetivoRobo[indGoleiro].y = TAM_Y_CAMPO / 2;
    objetivoRobo[indGoleiro].angulo = 0;
    objetivoRobo[indGoleiro].vel = 0;

    objetivoRobo[indVolante].x = 50;
    objetivoRobo[indVolante].y = TAM_Y_CAMPO / 2;
    objetivoRobo[indVolante].angulo = 0;
    objetivoRobo[indVolante].vel = 0;

    objetivoRobo[indAtacante].x = TAM_X_CAMPO / 2 - 20;
    objetivoRobo[indAtacante].y = TAM_Y_CAMPO / 2;
    objetivoRobo[indAtacante].angulo = 0;
    objetivoRobo[indAtacante].vel = 0;

    if (abs(estado[indGoleiro].x - objetivoRobo[indGoleiro].x) +
                abs(estado[indGoleiro].y - objetivoRobo[indGoleiro].y) <
            TAM_ROBO &&
        abs(estado[indVolante].x - objetivoRobo[indVolante].x) +
                abs(estado[indVolante].y - objetivoRobo[indVolante].y) <
            TAM_ROBO &&
        abs(estado[indAtacante].x - objetivoRobo[indAtacante].x) +
                abs(estado[indAtacante].y - objetivoRobo[indAtacante].y) <
            TAM_ROBO) {
      emPosiciona = false;
      emJogo = false;
    }
  }
  if (emInicio) {
    objetivoRobo[indGoleiro].x = TAM_X_DO_GOL + TAM_ROBO / 2;
    objetivoRobo[indGoleiro].y = TAM_Y_CAMPO / 2;
    objetivoRobo[indGoleiro].angulo = 0;
    objetivoRobo[indGoleiro].vel = 0;

    objetivoRobo[indVolante].x = TAM_X_CAMPO / 2 - 45;
    objetivoRobo[indVolante].y = TAM_Y_CAMPO / 2;
    objetivoRobo[indVolante].angulo = 0;
    objetivoRobo[indVolante].vel = 0;

    objetivoRobo[indAtacante].x = TAM_X_CAMPO / 2 - 20;
    objetivoRobo[indAtacante].y = TAM_Y_CAMPO / 2;
    objetivoRobo[indAtacante].angulo = 0;
    objetivoRobo[indAtacante].vel = 0;

    if (abs(estado[indGoleiro].x - objetivoRobo[indGoleiro].x) +
                abs(estado[indGoleiro].y - objetivoRobo[indGoleiro].y) <
            TAM_ROBO &&
        abs(estado[indVolante].x - objetivoRobo[indVolante].x) +
                abs(estado[indVolante].y - objetivoRobo[indVolante].y) <
            TAM_ROBO &&
        abs(estado[indAtacante].x - objetivoRobo[indAtacante].x) +
                abs(estado[indAtacante].y - objetivoRobo[indAtacante].y) <
            TAM_ROBO) {
      emInicio = false;
      emJogo = false;
    }
  }
}

extern int quadro;

//------------  E S T R A T E G I A -------------
void determinaEstadoJogo(){
  float xAtacante, yAtacante, xVolante, yVolante, xBola, yBola, dxBola, dyBola;
  
  xBola = estadoPrev[IND_BOLA].x;
  yBola = estadoPrev[IND_BOLA].y;
  dxBola = estadoPrev[IND_BOLA].dx;
  dyBola = estadoPrev[IND_BOLA].dy;

  xAtacante = estadoPrev[indAtacante].x;
  yAtacante = estadoPrev[indAtacante].y;
  xVolante = estadoPrev[indVolante].x;
  yVolante = estadoPrev[indVolante].y;

  // Se for tiro de meta
  if(tiroMeta)
    estadoDoJogo = TIRODEMETA;
  else if (emPenalidade)
    estadoDoJogo = PENALIDADE;

  // Atacante esta em posse da bola
  else if(isRoboEmPosseBola(indAtacante)){
    if(xVolante > xAtacante - 10){
      if (std::abs(yVolante - TAM_Y_CAMPO) < std::abs(yAtacante - TAM_Y_CAMPO) || xVolante > xAtacante)
        estadoDoJogo = ATAQUE_A;
      else
        estadoDoJogo = ATAQUE_B;
    }
    else
      estadoDoJogo = ATAQUE_B;
  }
  // Atacante perdeu a posse da bola e a bola esta indo para nosso gol
  else if(dxBola <= 0){
    if (xBola > 0.6 * TAM_X_CAMPO){
      estadoDoJogo = DEFESA_B;
    }
    else if (estadoDoJogo == DEFESA_B && xBola <= 0.4 * TAM_X_CAMPO){
      estadoDoJogo = DEFESA_A;
    }
    else {
      estadoDoJogo = DEFESA_A;
    }
  }
  
}

void estrategia(void) {
  unsigned char cmd[NUM_ROBOS_TIME * 2];
  int i;
  calculaPrevisao();
  determinaEstadoJogo();
  EstrGoleiro();
  estrVolante(indVolante);
  estrAtacante();

  if (trocarVolanteAtacante) {
    int tmp = indAtacante; // Robo vira atacante
    indAtacante = indVolante;
    indVolante = tmp;
    trocarVolanteAtacante = 0;
  }

  verificaPosiconamentos();

  for (i = 0; i < 3; i++) {
    int e, d, ea, da, cmdRE, cmdRD;

    // calculaWaveFront(objetivoRobo[i].x, objetivoRobo[i].y,
    //                  objetivoRobo[i].angulo, false, estadoPrev, mapa);

    // printf("atenção às linhas abaixo %d %s, devem ser retiradas no jogo\n",
    //        __LINE__, __FILE__);
    // objetivoRobo[2].x = 25;
    // objetivoRobo[2].y = 30;
    // objetivoRobo[2].vel = 7;
    // objetivoRobo[2].angulo = (quadro*10)%360;
    calculaCampoPotencial(i, objetivoRobo[i].x, objetivoRobo[i].y,
                          objetivoRobo[i].angulo, objetivoRobo[i].vel, true,
                          false, estadoPrev);

    calculaCmd(i, objetivoRobo[i].angulo, objetivoRobo[i].x, objetivoRobo[i].y,
               objetivoRobo[i].vel, cmdRE, cmdRD);

    //    CalculaCmd(i, 45, 150, 90, 0);
    ea = abs(e = cmdRE);
    if (e < 0)
      ea |= 0x8;
    da = abs(d = cmdRD);
    if (d < 0)
      da |= 0x8;
    cmd[i * 2] = ea;
    cmd[i * 2 + 1] = da;
  }
  //	cout << "!";
  //		enviaDados(0x11, 0x11, 0x11);
  // cmd[0] = cmd[2] = cmd[4] = 1;
  // cmd[1] = cmd[3] = cmd[5] = 1;
  // objetivoRobo[0].x = 30;

  for (int j = TAM_CMD_ENV - 1; j > 0; j--) {
    for (i = 0; i < 3; i++) {
      cmdEnviado[j][i] = cmdEnviado[j - 1][i];
    }
  }
  cmdEnviado[0][0].esq = (cmd[0] & 8) == 0 ? cmd[0] : -(cmd[0] & 0x7);
  cmdEnviado[0][0].dir = (cmd[1] & 8) == 0 ? cmd[1] : -(cmd[1] & 0x7);
  cmdEnviado[0][1].esq = (cmd[2] & 8) == 0 ? cmd[2] : -(cmd[2] & 0x7);
  cmdEnviado[0][1].dir = (cmd[3] & 8) == 0 ? cmd[3] : -(cmd[3] & 0x7);
  cmdEnviado[0][2].esq = (cmd[4] & 8) == 0 ? cmd[4] : -(cmd[4] & 0x7);
  cmdEnviado[0][2].dir = (cmd[5] & 8) == 0 ? cmd[5] : -(cmd[5] & 0x7);
  if (emJogo) {
    enviaDados(cmd[0] << 4, cmd[1] << 4, cmd[2] << 4, cmd[3] << 4, cmd[4] << 4,
               cmd[5] << 4);
    //    fprintf(fp, "%3d, %3x, %3x, %3x\n", ContQuadro, cmd[0], cmd[1],
    //    cmd[2]);
  } else {
    printf("G: %3x, V: %3x, A: %3x\n", indGoleiro, indVolante, indAtacante);
    printf("%3x, %3x, %3x, %3x, %3x, %3x\n", cmd[0], cmd[1], cmd[2], cmd[3],
           cmd[4], cmd[5]);
    enviaDados(0, 0, 0, 0, 0, 0);
  }
  // calculaCampoPotencial(1, objetivoRobo[1].x, objetivoRobo[1].y,
  //                         objetivoRobo[1].angulo, objetivoRobo[1].vel, true,
  //                         false, estadoPrev);
}
