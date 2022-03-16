#include "stm32l0xx.h"


//le port pour les caractères sera la port GPIOA
#define PORT_CHAR GPIOA

//on utilise 4 fils pour les caractères connectés sur PA6 à PA9
//les caracrtères seront envoyés à l'afficheur par groupes de 4 bits
#define DB7 9 //PA9
#define DB6 8 //PA8
#define DB5 7 //PA7
#define DB4 6 //PA6

//Les pins RS et E seront affectées au port B
#define Port_RS  GPIOB
#define Port_E  GPIOB
#define rs 4 //PB4
#define E 5 //PB5


void Ecrire(char f);

void toggle_e();

void D_set_E_Toggle (char f);

void EcrireFonction(char f);

void EcrireCaractere(char c);

void EcrireChaine(char ch[]);

void lcdinit4();

void Affichage_LCD();
