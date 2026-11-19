#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>

#define INF INT_MAX - 8
#define WHITE 0
#define GREY 1
#define BLACK 2

// la macro PARENT calcola l'indice nello heap di un dato nodo x. Osservo che non è necessario distinguere se x è pari o dispari, il risultato è lo stesso
#define PARENT(x) (x - 1) >> 1
// la macro MAX restituisce il massimo tra due interi
#define MAX(x, y) x > y ? x : y
// la macro PARTE_INTERA_INF resituisce la parte intera inferiore di un float
#define PARTE_INTERA_INF(x) x < 0 ? (int) (x - 1) : (int) x 

// la struct nodo è definita come struct di bitfield

typedef struct TDAnode {
	unsigned int AR_ref : 12;	// con 3000 possibili slot nel registro, sono sufficienti 12 bit per indirizzarli tutti
	unsigned int weight : 7;	// il costo massimo di un esagono è 100, per cui sono sufficienti 7 bit
	unsigned int AR_num : 3;	// il numero di rotte aeree massimo è 5, per cui sono sufficienti 5 bit
	unsigned int col : 2;		// ci sono 3 colori possibili, dunque sono sufficienti 2 bit
	char padding;				// il padding fa in modo che la struct occupi 32 bit (4 byte)
} Node;

// la dimensione del tipo Nodo fa in modo tale che in una parola di cache entrino 16 nodi interamente, cosicché
// tutti i campi siano presenti contemporaneamente


typedef struct TDAheap {
	int dist;
	int pos;
} Queue_entry;

// tipo Queue_entry, usato nella coda di BFS, in change_cost

typedef Queue_entry* Queue_t;

typedef struct { 
	int dist;
	int map_ref;
} Heap_entry;

// tipo Heap_entry, usato nello heap di Dijkstra, in travel_cost

typedef Heap_entry* Heap_t;

// i due tipi Queue_entry e Heap_entry sono praticamente identici, ma definiti diversamente per facilitare la lettura del codice

typedef struct returnPack {
	Node* map;
	int* nei;
	int* dist;
	int* posInHeap;
} ret_Packet_t;

// struct d'appoggio per aggiornare tramite init i vettori map, neighbours, dist e posInHeap

																/* FUNZIONI AUSILIARIE */


void swap(Heap_t Heap, int* posInHeap, int pos1, int pos2) {

	// swap: inverte due elementi (nello heap)
	
	Heap_entry tmp = {0, 0};		// appoggio per lo swap
	
	tmp = Heap[pos1];
	Heap[pos1] = Heap[pos2];
	Heap[pos2] = tmp;
	posInHeap[(Heap[pos1].map_ref)] = pos1;
	posInHeap[(Heap[pos2].map_ref)] = pos2;
}

void climb_up(Heap_t Heap, int* posInHeap, int index) {

	// climb_up: viene chiamata quando aggiorno la distanza di un nodo con una minore.
	// Scambia tale nodo con suo padre finché quest'ultimo non è minore
	
	int parent = 0;

	/*
		non è necessario controllare che parent rimanga positivo:
		se index > 0, parent non può mai essere negativo
		se index = 0, non entro mai nel ciclo o esco nel momento in cui lo diventa
	*/
	
	while (index > 0) {									
		parent = PARENT(index);							// chiamo la macro PARENT
		if (Heap[parent].dist <= Heap[index].dist)		// se parent ha dist minore del figlio, esco dal ciclo
			break;
		swap(Heap, posInHeap, index, parent);
		index = parent;									// aggiorno index con suo padre
	}
}

void Min_Heapify_perDijkstra(Heap_t Heap, int* posInHeap, int heapSize, int index) {

	// Min_Heapify_perDijkstra: viene chiamata da cancellaMin.
	// scambia index con il minore dei suoi due figli, se esistono
	
	int left = 0;					// figlio sinistro
	int right = 0;					// figlio destro
	int posmin = index;				// posizione del nodo a dist minima tra index, left e right

	while (1) {						// loop infinito
		left = (index << 1) + 1;	// calcolo l'indice del figlio sx
		right = left + 1;			// l'indice del figlio dx sarebbe (index << 1) + 2, che è uguale a left + 1
		if (left < heapSize && Heap[left].dist < Heap[posmin].dist)
			posmin = left;			// il figlio sx esiste e ha dist minore del minimo precedente
		if (right < heapSize && Heap[right].dist < Heap[posmin].dist)
			posmin = right;			// il figlio dx esiste e ha dist minore del minimo precedente
		if (posmin == index)
			break;					// nessuno dei due figli è minore di index, esco dal ciclo
		swap(Heap, posInHeap, index, posmin);
		index = posmin;				// aggiorno index con il minimo precedente
	}
}

void push(Heap_t Heap, int* posInHeap, int* heapSizeP, int index, int dist) {

	// push: inserisco index nello heap, con distanza dist
	
	int appoggio = 0;

	appoggio = *heapSizeP;
	(*heapSizeP)++;							// incremento heapSize
	Heap[appoggio].map_ref = index;			// inserisco index in ultima posizione
	Heap[appoggio].dist = dist;
	posInHeap[index] = appoggio;
	climb_up(Heap, posInHeap, appoggio);	// faccio salire index al suo posto
}

Heap_entry cancellaMin(Heap_t Heap, int* posInHeap, int* heapSizeP) {

	// cancellaMin: restituisce l'elemento minimo dello heap (la radice)
	
	Heap_entry min = {0, 0};
	int size = 0;

	min = Heap[0];
	posInHeap[min.map_ref] = -1;		// segnalo che min è stato rimosso dallo heap

	(*heapSizeP)--;						// decremento heapSize
	size = *heapSizeP;
	if (size > 0) {						// se lo heap non è vuoto, metto l'ultimo elemento in radice e chiamo Min_Heapify_perDijkstra
		Heap[0] = Heap[size];
		posInHeap[(Heap[0].map_ref)] = 0;
		Min_Heapify_perDijkstra(Heap, posInHeap, size, 0);
	}
	return min;
}


																/* FUNZIONI PRINCIPALI */


ret_Packet_t init(Node* map, int* neighbours, int totNodes, int nCol, int nRow, int* dist, int* posInHeap) {

	// init: Inizializza i vettori map, neighbours, dist e posInHeap. 
	
	int adj_index = 0;		// indice dell'adiacente (per neighbours)
	int cur_row = 0;		// indice di riga dell'elemento corrente
	int cur_col = 0;		// indice di colonna dell'elemento corrente
	int adj_col = 0;		// indice di colonna dell'adiacente
	int adj_row = 0;		// indice di riga dell'adiacente
	int neighbour_0 = 0;	// indice dell'inizio della pagina degli adiacenti

	// due matrici che contengono gli spiazzamenti per calcolare la posizione dei vicini
	
	short int even_offset[6][2] = {
		{-1, 0}, {1, 0}, {-1, -1}, {0, -1}, {-1, 1}, {0, 1}
	};
	short int odd_offset[6][2] = {
		{-1, 0}, {1, 0}, {0, -1}, {1, -1}, {0, 1}, {1, 1}
	};
	ret_Packet_t returning = {NULL, NULL, NULL, NULL};			// dati di ritorno

	map = (Node*) malloc(totNodes * sizeof(Node)); 				// alloco map
	neighbours = (int*) calloc(6 * totNodes, sizeof(int));		// alloco neighbours
	dist = (int*) malloc(totNodes * sizeof(int));				// alloco dist
	posInHeap = (int*) malloc(totNodes * sizeof(int));			// alloco posInHeap

	for (int matrixIndex = 0; matrixIndex < totNodes; matrixIndex++) {
		map[matrixIndex].weight = 1;							// costo d'uscita in iniziale = 1
		map[matrixIndex].AR_num = 0;							// numero di rotte aeree iniziale = 0
		map[matrixIndex].AR_ref = 3002;							// riferimento al registro di rotte aeree iniziale = 3002
		map[matrixIndex].col = WHITE;							// colore iniziale = WHITE
		cur_row = PARTE_INTERA_INF(matrixIndex / nCol);			// uso la macro definita sopra per ricavare l'indice di riga dell'esagono corrente
		cur_col = matrixIndex % nCol;							// ricavo l'indice di colonna dell'esagono corrente
		neighbour_0 = 6 * matrixIndex;
		for (int i = 0; i < 6; i++) {
			if (cur_row % 2 == 0) {
				adj_col = cur_col + even_offset[i][0];
				adj_row = cur_row + even_offset[i][1];			// applico l'offset nel caso di riga pari
			}
			else {
				adj_col = cur_col + odd_offset[i][0];
				adj_row = cur_row + odd_offset[i][1];			// applico l'offset nel caso di riga dispari
			}
			if (adj_col < 0 || adj_col >= nCol || adj_row < 0 || adj_row >= nRow)
				adj_index = INF;								// se l'adiacente ricavato ha coordinate fuori mappa, imposto l'indice a INF
			else
				adj_index = adj_row * nCol + adj_col;
			neighbours[neighbour_0 + i] = adj_index;			// inserisco l'indice calcolato in neighbours
		}
		dist[matrixIndex] = INF;								// distanza iniziale da sourcePos = INF
		posInHeap[matrixIndex] = -1;							// posizione iniziale nello heap = -1 (assente)
	}
	returning.map = map;
	returning.nei = neighbours;
	returning.dist = dist;
	returning.posInHeap = posInHeap;

	// a questo punto, l'inizializzazione è andata a buon fine, stampo OK e ritorno.
	
	printf("OK\n");
	return returning;
}

void travel_cost(int sourcePos, int destPos, int totNodes, Node* map, int* neighbours, int* AR_reg, int* dist, int* posInHeap) {

	// travel_cost: implementa l'algoritmo di Dijkstra con min-heap per ricercare il percorso a costo minimo da sourcePos a qualsiasi altro nodo
	
	Heap_t Q = NULL;				// lo heap Q
	Heap_entry u = {0, 0};			// struct che immagazzina il nodo a dist minima estratto dallo heap
	int* modificati = NULL;			// vettore che tiene traccia di tutti gli esagoni visitati, così da velocizzare il reset.
	int modificati_count = 0;
	int heapSize = 0;				// indice di popolamento dello heap
	int curr_pos = 0;				// indice dell'elemento corrente in map
	int curr_dist = 0;				// distanza dell'elemento corrente da sourcePos
	int nuovadist = 0;				// nuova distanza che tiene conto del costo d'uscita da curr
	int neighbour_pos = 0;			// indice del vicino
	int neighbour_0 = 0;			// riferimento alla "pagina" dei vicini in neighbours
	int AR_num = 0;					// numero di rotte aeree uscenti da curr
	int AR_ref = 0;					// riferimento alla "pagina" delle destinzioni in AR_reg
	int AR_neighbour_pos = 0;		// indice in map della destinazione di una rotta
	
	if (map[sourcePos].weight == 0) {
		printf("-1\n");				// se l'esagono di partenza ha costo nullo, non posso uscire
		return;
	}

	Q = (Heap_t) malloc(totNodes * sizeof(Heap_entry));
	modificati = calloc(totNodes, sizeof(int));

	// inserisco sourcePos nello heap
	
	dist[sourcePos] = 0;
	push(Q, posInHeap, &heapSize, sourcePos, 0);
	modificati[modificati_count++] = sourcePos;

	while (heapSize > 0) {
		u = cancellaMin(Q, posInHeap, &heapSize);			// rimuovo l'esagono a dist minima
		curr_pos = u.map_ref;
		curr_dist = u.dist;

		if (curr_pos == destPos) {							// se ho estratto l'esagono di destinazione, vuol dire che l'ho già raggiunto con il percorso a costo minimo
			printf("%d\n", curr_dist);
			for (int i = 0; i < modificati_count; i++) {
				curr_pos = modificati[i];
				dist[curr_pos] = INF;
				posInHeap[curr_pos] = -1;					// reset di dist e posInHeap
			}
			free(Q);
			free(modificati);
			return;
		}

		if (map[curr_pos].weight == 0)						// se l'esagono corrente ha costo nullo, non posso uscire, passo al successivo
			continue;

		nuovadist = curr_dist + map[curr_pos].weight;		// calcolo la nuova distanza
		neighbour_0 = 6 * curr_pos;							// indice della "pagina" dei vicini in neighbours

		for (int i = 0; i < 6; i++) {
			neighbour_pos = neighbours[neighbour_0 + i];	// ottengo l'indice del vicino
			if (neighbour_pos == INF)
				continue;									// se ha valore iniziale (INF), allora non esiste, passo al successivo
			if (map[neighbour_pos].weight == 0 && neighbour_pos != destPos)
				continue;									// se ha costo nullo e NON è la destinazione, posso saltarlo (vicolo cieco)

			if (nuovadist < dist[neighbour_pos]) {
				dist[neighbour_pos] = nuovadist;			// aggiorno distanza
				if (posInHeap[neighbour_pos] >= 0) {		// se presente nello heap, aggiorno la priorità (dist)
					Q[(posInHeap[neighbour_pos])].dist = nuovadist;
					climb_up(Q, posInHeap, posInHeap[neighbour_pos]);
				}
				else {
					push(Q, posInHeap, &heapSize, neighbour_pos, nuovadist);		// inserisco nello heap in caso assente
					modificati[modificati_count++] = neighbour_pos;
				}
			}
		}

		AR_num = map[curr_pos].AR_num;
		if (AR_num == 0)
			continue;										// se non escono rotte aeree da curr, passo al successivo
		AR_ref = map[curr_pos].AR_ref;

		for (int j = 0; j < AR_num; j++) {
			AR_neighbour_pos = AR_reg[AR_ref + j];
			if (AR_neighbour_pos != destPos && map[AR_neighbour_pos].weight == 0)
				continue;									// se la rotta conduce a un vicolo cieco e NON alla destinazione finale, salto l'arrivo
			if (nuovadist < dist[AR_neighbour_pos]) {
				dist[AR_neighbour_pos] = nuovadist;			// aggiorno distanza
				if (posInHeap[AR_neighbour_pos] >= 0) {		// se presente nello heap, aggiorno la priorità (dist)
					Q[(posInHeap[AR_neighbour_pos])].dist = nuovadist;
					climb_up(Q, posInHeap, posInHeap[AR_neighbour_pos]);
				}
				else {
					push(Q, posInHeap, &heapSize, AR_neighbour_pos, nuovadist);		// inserisco nello heap in caso assente
					modificati[modificati_count++] = AR_neighbour_pos;
				}
			}
		}
	}

	// a questo punto, non ho estratto il nodo destinazione, il che vuol dire che è irraggiungibile
	
	printf("-1\n");

	
	for (int i = 0; i < modificati_count; i++) {
		curr_pos = modificati[i];
		dist[curr_pos] = INF;
		posInHeap[curr_pos] = -1;			// reset di dist e posInHeap
	}
	
	free(Q);
	free(modificati);
}


int* toggle_air_route(Node* map, int sourcePos, int destPos, int* last_empty, int* AR_reg) {

	// toggle_air_route: inizializza, se necessario, il registro delle rotte aeree, e vi aggiunge opportunamente le destinazioni.
	// Ad ogni esagono sourcePos su cui viene chiamata la fuzione, è assegnata una "pagina" di 5 int all'interno di AR_reg, abbastanza
	// per alloggiare gli indici delle destinazioni di tutte le possibili rotte aeree uscenti da sourcePos.
	
	int i = 0;					// sentinella di un while
	int AR_ref_src = 0;			// riferimento alla "pagina" delle rotte aeree di sourcePos
	int AR_num = 0;				// numero delle rotte aree di sourcePos

	if (map[sourcePos].AR_num == 5) {
		printf("KO\n");			// sono già state inizializzate 5 rotte aeree per sourcePos
		return AR_reg;
	}

	// se non esiste ancora, alloco lo spazio per il registro 
	
	if (AR_reg == NULL) {
		AR_reg = (int*) malloc(3000 * sizeof(int));
		for (int j = 0; j < 3000; j++) {
			AR_reg[j] = -1;
		}
	}

	/*
		3000 è calcolato osservando che in ogni test pubblico, il numero di chiamate a toggle_air_route non eccede 400. Nel caso pessimo in cui ogni chiamata sia ad un esagono sorgente differente,
		riservo 5 celle per ciascun esagono * 400 esagoni diversi = 2000. Per stare sicuri, aggiungo altri 1000 spazi. Essendo 3000 int, l'impatto sullo spazio occupato è trascurabile rispetto
		al resto del programma.
	*/

	if (map[sourcePos].AR_ref > 3000) {								// il campo AR_ref di sourcePos è inizializzato al valore 3002 (vedi init)
		AR_reg[*last_empty] = destPos;								// se corrisponde al valore di inizializzazione, inserisco.
		map[sourcePos].AR_ref = *last_empty;						// il riferimento in map[sourcePos] è alla prima destinazione nella "pagina" di sourcePos
		map[sourcePos].AR_num++;
		(*last_empty) += 5;											// aggiorno il riferimento all'ultima pagina vuota
		while (*last_empty < 3000 && AR_reg[*last_empty] != -1)
			(*last_empty) += 5;

		// essendo una funzione toggle, è possibile che, rimuovendo dei riferimenti, nascano dei buchi nel registro, per cui
		// con il while, scorro il registro finché non trovo una pagina vuota

		if (*last_empty >= 3000)
			return AR_reg;					// se non ho trovato una pagina vuota, vuol dire che il registro è pieno, andrebbe riallocato successivamente.

		// Tuttavia, per quanto detto prima, è trascurabile il problema di riallocare il registro in quanto c'è sufficiente spazio per il numero di chiamate stimato.
		// A questo punto l'inserimento è andato a buon fine, stampo OK e ritorno
		
		printf("OK\n");
		return AR_reg;
	}
	else {												// il riferimento in map[sourcePos] è presente, vuol dire che sono già state inizializzate rotte da sourcePos
		AR_ref_src = map[sourcePos].AR_ref;
		AR_num = map[sourcePos].AR_num;					// ricavo il numero di rotte aeree già esistenti, uscenti da sourcePos
		while (i < AR_num) {							// le controllo tutte
			if (AR_reg[AR_ref_src + i] == destPos) {	// se la rotta che devo inserire è già presente, devo rimuoverla
				if (i != AR_num - 1)
					AR_reg[AR_ref_src + i] = AR_reg[AR_ref_src + AR_num - 1];

				// con questo swap, compatto la pagina "a sinistra", così da non lasciare buchi 
				// non è rilevante l'ordine in cui sono immagazzinate le destinazioni
				
				AR_reg[AR_ref_src + AR_num - 1] = -1;	// imposto la cella a vuota (-1)
				map[sourcePos].AR_num--;
				if (map[sourcePos].AR_num == 0) {
					map[sourcePos].AR_ref = 3002;		// se ho rimosso l'unica/ultima rotta aerea, azzero il riferimento in map[sourcePos] e aggiorno last_empty
					(*last_empty) = AR_ref_src;
				}

				// a questo punto, l'operazione è andata a buon fine, stampo OK e ritorno
				printf("OK\n");
				return AR_reg;
			}
			i++;
		}

		// a questo punto, la rotta in input è assente nel registro, per cui la inserisco
		
		AR_reg[AR_ref_src + AR_num] = destPos;
		map[sourcePos].AR_num++;
		printf("OK\n");
		
		return AR_reg;
	}
}

void change_cost(Node* map, int sourcePos, int value, int radius, int* neighbours) {

	// change_cost: implemento BFS per cambiare i costi degli esagoni a macchia d'olio
	
	Queue_t Queue = NULL;				// coda di supporto per BFS
	Queue_entry curr = {0, 0};			// appoggio per l'elemento corrente
	int weight = 0;						// per limitare il costo d'uscita a un int compreso tra 0 e 100
	float appoggio = 0;
	int Queuelength = 0;				// la lunghezza dello stoccaggio della coda
	int head = 0;						// indice della testa
	int tail = 0;						// indice della coda
	int QueuePop = 0;					// popolazione della coda: quanti elementi sono presenti
	int cur_pos = 0;					// riferimento alla posizione in map dell'esagono corrente
	int curr_dist = 0;					// distanza dell'esagono corrente da sourcePos
	int adj_index = 0;					// indice in map di un adiacente dell'esagono corrente
	int neighbour_0 = 0;				// riferimento alla "pagina" degli adiacenti dell'esagono corrente (vedi init)
	int nuovadist = 0;					// nuova distanza considerando il costo d'uscita dall'esagono corrente

	// aggiorno il costo d'uscita dell'esagono con indice sourcePos
	map[sourcePos].col = GREY;
	weight = map[sourcePos].weight + value;
	if (weight < 0)
		weight = 0;
	if (weight > 100)
		weight = 100;
	map[sourcePos].weight = weight;

	Queuelength = 1 + (3 * radius * (radius + 1));		// calcolo quante celle dovrò allocare per la coda

	/*
		osservo che ad ogni cambio di distanza (dist) dall'origine (sourcePos), inserisco nella coda 6 * dist esagoni:
		a distanza 0 ho 1 esagono (sourcePos stesso)
		a distanza 1 ho 6 esagoni (gli adiacenti diretti di sourcePos)
		a distanza 2 ho 12 esagoni (gli adiacenti degli adiacenti)
		...
		a distanza r ho 6 * r esagoni

		1 + sum(n = 1, n <= r) { 6n } = 1 + 6 * sum(n = 1, n <= r) { n } = 1 + 6 * ((r * (r + 1)) / 2)
		casting a int non è necessario poiché il risultato è sempre un intero
	*/

	Queue = (Queue_t) malloc(Queuelength * sizeof(Queue_entry));
	for (int i = 0; i < Queuelength; i++) {
		Queue[i].dist = INF;
		Queue[i].pos = -1;						// inizializzazione della coda
	}
	Queue[0].dist = 0;
	Queue[0].pos = sourcePos;
	QueuePop = 1;
	tail++;										// inserisco sourcePos in coda

	while (QueuePop > 0) {
		curr = Queue[head];
		head++;
		QueuePop--;								// rimuovo la testa della coda (incremento l'indice della testa)
		cur_pos = curr.pos;
		curr_dist = curr.dist;
		nuovadist = curr_dist + 1;
		neighbour_0 = 6 * cur_pos;
		for (int i = 0; i < 6; i++) {

			adj_index = neighbours[neighbour_0 + i];
			if (adj_index == INF)
				continue;						// se non esiste il vicino (vedi init), passo al successivo
			if (map[adj_index].col == WHITE && curr_dist < radius) {
				map[adj_index].col = GREY;
				
				appoggio = radius - nuovadist;
				appoggio = appoggio / (float) radius;
				appoggio = MAX(0, appoggio);
				appoggio = ((float) value) * appoggio;
				appoggio = PARTE_INTERA_INF(appoggio);
				appoggio = map[adj_index].weight + appoggio;
				if (appoggio < 0)
					appoggio = 0;
				if (appoggio > 100)
					appoggio = 100;
				map[adj_index].weight = appoggio; // il procedimento è separato per semplicità di debugging
				
				Queue[tail].dist = nuovadist;
				Queue[tail].pos = adj_index;
				tail++;
				QueuePop++;						// aggiungo in coda il vicino appena esaminato
			}
		}
		map[cur_pos].col = BLACK;				// finiti i vicini, coloro di nero l'elemento corrente e passo al successivo in coda
	}

	for (int i = 0; i < Queuelength; i++) {
		cur_pos = Queue[i].pos;
		if (cur_pos != -1) {
			map[cur_pos].col = WHITE;			// ripristino il campo colore di map
		}
	}

	// a questo punto, la procedura è andata a buon fine, stampo OK e libero la coda, poi ritorno al main
	
	printf("OK\n");
	free(Queue);
	return;
}




int main() {

	// main: leggo il file in input finché non termina (Fcontrol == EOF), ad ogni comando letto chiamo la funzione corrispondente.
	

	// piccola struct di supporto per i return values di init
	ret_Packet_t returned = {NULL, NULL, NULL, NULL};
	char* comando = NULL;			// stringa di supporto per identificare il comando in input
	Node* map = NULL; 				// vettore mappa (vedi init())
	int* AR_reg = NULL; 			// vettore registro delle rotte aeree (vedi toggle_air_route())
	int* neighbours = NULL; 		// matrice di vicini (vedi init())
	int* dist = NULL;
	int* posInHeap = NULL;			// vettori di supporto per travel_cost()
	int Fcontrol = 1; 				// EOF è un int negativo, pertanto inizializzo la sentinella a un qualsiasi intero positivo.

	int nCol = 0;
	int nRow = 0;					// dimensioni della tabella
	int sourcePos = 0;				// indice in map del nodo di partenza (travel_cost e change_cost)
	int destPos = 0;				// indice in map del nodo destinazione (travel_cost)
	int inputPit = 0;
	int val = 0;
	int rad = 0;					// variabili in uso da change_cost
	int cS = 0;
	int rS = 0;						// per immagazzinare le coordinate del nodo sorgente
	int last_empty = 0;				// riferimento all'ultima "pagina" vuota nel registro rotte aeree
	int cD = 0;
	int rD = 0;						// per immagazzinare le coordinate del nodo destinazione
	int totNodes = 0;				// numero totale di nodi

	comando = (char*) malloc(17 * sizeof(char)); 			// il comando più lungo da leggere è toggle_air_route, che comprendendo il terminatore è lungo 17 caratteri
	while (Fcontrol != EOF) {
		Fcontrol = scanf("%s", comando); 					// scansiono il file di input dall'inizio finchè non trovo uno spazio, ergo memorizzo il nome del comando in input
		if (!strcmp("init", comando)) {
			comando = strcpy(comando, "");					// svuoto la stringa 'comando'
			if (map != NULL) {
				free(map);
				free(dist);
				free(posInHeap);
				free(neighbours);							// rimuovo dati precedentemente inizializzati, se presenti
			}
			inputPit = scanf("%d %d", &nCol, &nRow);		// leggo le nuove dimensioni
			totNodes = nCol * nRow;
			returned = init(map, neighbours, totNodes, nCol, nRow, dist, posInHeap);					// call a init()
			map = returned.map;
			neighbours = returned.nei;
			dist = returned.dist;
			posInHeap = returned.posInHeap;					// aggiorno i vettori di servizio
		}
		
		// il resto delle casistiche viene gestito in maniera analoga
		
		if (!strcmp("change_cost", comando)) {
			comando = strcpy(comando, "");
			inputPit = scanf("%d %d %d %d", &cS, &rS, &val, &rad);
			if (cS < 0 || cS > nCol - 1 || rS < 0 || rS > nRow - 1 || map == NULL || rad == 0 || val < -10 || val > 10) {
				printf("KO\n");			// verifico che i dati in ingresso siano accettabili e/o che esista una mappa su cui chiamare le funzioni
				continue;
			}
			sourcePos = rS * nCol + cS;
			change_cost(map, sourcePos, val, rad, neighbours);
		}
		if (!strcmp("toggle_air_route", comando)) {
			comando = strcpy(comando, "");
			inputPit = scanf("%d %d %d %d", &cS, &rS, &cD, &rD);
			if (cS < 0 || cS > nCol - 1 || rS < 0 || rS > nRow - 1 || cD < 0 || cD > nCol - 1 || rD < 0 || rD > nRow - 1 || map == NULL) {
				printf("KO\n");
				continue;
			}
			sourcePos = rS * nCol + cS;
			destPos = rD * nCol + cD;
			AR_reg = toggle_air_route(map, sourcePos, destPos, &last_empty, AR_reg);
		}
		if (!strcmp("travel_cost", comando)) {
			comando = strcpy(comando, "");
			inputPit = scanf("%d %d %d %d", &cS, &rS, &cD, &rD);
			if (cS < 0 || cS > nCol - 1 || rS < 0 || rS > nRow - 1 || cD < 0 || cD > nCol - 1 || rD < 0 || rD > nRow - 1 || map == NULL) {
				printf("-1\n");
				continue;
			}
			sourcePos = rS * nCol + cS;
			destPos = rD * nCol + cD;
			travel_cost(sourcePos, destPos, totNodes, map, neighbours, AR_reg, dist, posInHeap);
		}
	}

	// a questo punto, il file di input è concluso, libero la memoria allocata e termino
	
	free(comando);
	free(map);
	free(dist);
	free(posInHeap);
	free(AR_reg);
	free(neighbours);

	inputPit += 0;
	return 0;
}
