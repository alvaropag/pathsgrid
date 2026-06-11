#include <iostream>
#include <chrono>
#include <string>
#include <vector>
using namespace std;

// Descomente para imprimir cada poda com sua condição e o estado do grid
#define PRINT_DEBUG

// Tamanho do tabuleiro: N x N células (7x7 = 49 células no total)
const int N = 7;

// grid[r][c] = true se a célula (r,c) já foi visitada pelo caminho atual
bool grid[N][N];

#ifdef PRINT_DEBUG
static void printGrid(int linha, int coluna, const char* motivo) {
    printf("[PODA] %s  pos=(%d,%d)\n", motivo, linha, coluna);
    printf("  ");
    for (int c = 0; c < N; c++) printf("%d ", c);
    printf("\n");
    for (int r = 0; r < N; r++) {
        printf("%d ", r);
        for (int c = 0; c < N; c++)
            printf("%c ", grid[r][c] ? '#' : '.');
        printf("\n");
    }
    printf("\n");
}
#endif

// grau[r][c] = número de vizinhos NÃO visitados da célula (r,c) no estado atual.
// Mantido de forma incremental: atualizado a cada visita/desvisita, evitando
// recomputar do zero a cada chamada recursiva
int grau[N][N];

// Contadores globais de células em estado crítico (excluindo o destino [6][0]):
//   global_forcados: células com grau == 1 (beco: só uma saída disponível)
//   global_isolados: células com grau == 0 (ilhadas: sem nenhuma saída)
int global_forcados = 0;
int global_isolados = 0;

int  caminhos = 0;       // total de caminhos encontrados
long long totalChamadas = 0;
string padrao;           // string de 48 chars (D/U/L/R/?) que restringe cada passo

// Caminho em construção: caminhoAtual[i] = {linha, coluna} do i-ésimo passo (0-indexado)
pair<int,int> caminhoAtual[N * N];
// Lista de todos os caminhos hamiltonianos encontrados
vector<vector<pair<int,int>>> todosCaminhos;

// Vetores de direção: baixo, cima, direita, esquerda
const int DR[] = {1, -1, 0, 0};
const int DC[] = {0, 0, 1, -1};

inline bool isDest(int r, int c)   { return r == N - 1 && c == 0; }
inline bool inBounds(int r, int c) { return r >= 0 && r < N && c >= 0 && c < N; }

// Verifica se o padrão permite o movimento 'dir' no passo atual.
// '?' aceita qualquer direção; caso contrário, deve bater exatamente.
inline bool permitido(int passo, char dir) {
    return padrao[passo] == '?' || padrao[passo] == dir;
}

// --- OTIMIZAÇÃO 4: Verificação de conectividade geral (BFS) ---
// Faz um BFS a partir de (r0, c0) pelas células não visitadas e retorna
// quantas são alcançáveis. Se esse número for menor que o total de células
// ainda não visitadas, o tabuleiro está fragmentado: é impossível completar
// um caminho hamiltoniano a partir daqui, então o ramo é podado.
// Custo: O(N²) por chamada — é o gargalo principal após as outras otimizações.
inline int contarAlcancaveis(int r0, int c0) {
    bool vis[N][N] = {};
    int qr[N * N], qc[N * N];
    int head = 0, tail = 0, count = 0;
    qr[tail] = r0; qc[tail] = c0; tail++;
    vis[r0][c0] = true;
    while (head < tail) {
        int r = qr[head], c = qc[head]; head++;
        count++;
        for (int d = 0; d < 4; d++) {
            int nr = r + DR[d], nc = c + DC[d];
            if (!inBounds(nr, nc) || grid[nr][nc] || vis[nr][nc]) continue;
            vis[nr][nc] = true;
            qr[tail] = nr; qc[tail] = nc; tail++;
        }
    }
    return count;
}

// --- GRAU INCREMENTAL: visitar(r, c) ---
// Marca (r,c) como visitada e atualiza incrementalmente:
//   1. Remove (r,c) dos contadores globais se era grau 0 ou 1.
//   2. Decrementa o grau de cada vizinho não visitado de (r,c),
//      ajustando global_forcados e global_isolados conforme a transição:
//        grau 2 → 1: vizinho virou beco (global_forcados++)
//        grau 1 → 0: vizinho virou ilhado (global_forcados--, global_isolados++)
// Custo: O(4) — constante, independente do tamanho do tabuleiro.
void visitar(int r, int c) {
    grid[r][c] = true;
    if (!isDest(r, c)) {
        if      (grau[r][c] == 1) global_forcados--;
        else if (grau[r][c] == 0) global_isolados--;
    }
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (!inBounds(nr, nc) || grid[nr][nc]) continue;
        grau[nr][nc]--;
        if (!isDest(nr, nc)) {
            if      (grau[nr][nc] == 1) global_forcados++;
            else if (grau[nr][nc] == 0) { global_forcados--; global_isolados++; }
        }
    }
}

// --- GRAU INCREMENTAL: desvisitar(r, c) ---
// Desfaz exatamente o que visitar() fez (backtracking).
// Restaura grau dos vizinhos e os contadores globais ao estado anterior.
// Custo: O(4).
void desvisitar(int r, int c) {
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (!inBounds(nr, nc) || grid[nr][nc]) continue;
        grau[nr][nc]++;
        if (!isDest(nr, nc)) {
            if      (grau[nr][nc] == 2) global_forcados--;
            else if (grau[nr][nc] == 1) { global_forcados++; global_isolados--; }
        }
    }
    grid[r][c] = false;
    if (!isDest(r, c)) {
        if      (grau[r][c] == 1) global_forcados++;
        else if (grau[r][c] == 0) global_isolados++;
    }
}

void resolver(int linha, int coluna, int contagem) {
    totalChamadas++;

    // Caso base: todas as 49 células foram visitadas.
    // O caminho é válido apenas se terminou no destino [6][0].
    if (contagem == N * N) {
        if (isDest(linha, coluna)) {
            caminhos++;
            todosCaminhos.emplace_back(caminhoAtual, caminhoAtual + N * N);
        }
        return;
    }

    // --- OTIMIZAÇÃO 2: Chegada prematura ao destino ---
    // Se chegamos ao destino antes de visitar todas as células, este ramo
    // não pode completar um caminho hamiltoniano — descarta imediatamente.
    if (isDest(linha, coluna)) {
#ifdef PRINT_DEBUG
        printGrid(linha, coluna, "OPT2: destino prematuro");
#endif
        return;
    }

    // --- OTIMIZAÇÃO 5a: Verificação de célula isolada — O(1) ---
    // Se global_isolados > 0, existe alguma célula sem nenhum vizinho livre.
    // Ela nunca poderá ser visitada → impossível completar o caminho.
    if (global_isolados > 0) {
#ifdef PRINT_DEBUG
        printGrid(linha, coluna, "OPT5a: célula isolada");
#endif
        return;
    }

    // --- OTIMIZAÇÃO 5b: Verificação de beco sem saída — O(4) ---
    // Uma célula com grau 1 (só uma saída) só pode ser visitada como
    // último passo daquele corredor, ou seja, deve ser o destino final
    // ou o próximo passo a partir da posição atual.
    //
    // Contamos quantas células com grau 1 são vizinhas da posição atual
    // (adj_deg1): estas ainda podem ser "salvas" como primeiro passo.
    // As demais (global_forcados - adj_deg1) são becos inalcançáveis → poda.
    // Se adj_deg1 > 1: dois becos adjacentes — apenas um pode ser o próximo
    // passo, o outro virará inacessível → poda.
    if (global_forcados > 0) {
        int adj_deg1 = 0;
        for (int d = 0; d < 4; d++) {
            int nr = linha + DR[d], nc = coluna + DC[d];
            if (!inBounds(nr, nc) || grid[nr][nc] || isDest(nr, nc)) continue;
            if (grau[nr][nc] == 1) adj_deg1++;
        }
        if (global_forcados - adj_deg1 > 0) {
#ifdef PRINT_DEBUG
            printGrid(linha, coluna, "OPT5b: beco inalcançável (forcados > adj_deg1)");
#endif
            return;
        }
        if (adj_deg1 > 1) {
#ifdef PRINT_DEBUG
            printGrid(linha, coluna, "OPT5b: dois becos adjacentes (adj_deg1 > 1)");
#endif
            return;
        }
    }

    // --- OTIMIZAÇÃO 4: Conectividade geral — O(N²) ---
    // BFS a partir do destino [6][0]: conta quantas células não visitadas
    // são alcançáveis. Se for menos que o total restante, o tabuleiro está
    // fragmentado e nenhum caminho hamiltoniano é possível daqui.
    // O BFS parte do destino para garantir também que o destino em si
    // seja alcançável a partir das demais células.
    {
        int alcancaveis = contarAlcancaveis(N - 1, 0);
        int esperados   = N * N - contagem;
        if (alcancaveis != esperados) {
#ifdef PRINT_DEBUG
            char motivo[128];
            snprintf(motivo, sizeof(motivo),
                     "OPT4: fragmentação (alcancaveis=%d esperados=%d contagem=%d)",
                     alcancaveis, esperados, contagem);
            printGrid(linha, coluna, motivo);
#endif
            return;
        }
    }

    // Tenta cada movimento permitido pelo padrão na posição atual.
    // visitar/desvisitar mantém grau e contadores em sincronia (backtracking).
    int passo = contagem - 1;

    if (permitido(passo, 'D') && linha + 1 < N && !grid[linha + 1][coluna]) {
        visitar(linha + 1, coluna);
        caminhoAtual[contagem] = {linha + 1, coluna};
        resolver(linha + 1, coluna, contagem + 1);
        desvisitar(linha + 1, coluna);
    }
    if (permitido(passo, 'U') && linha - 1 >= 0 && !grid[linha - 1][coluna]) {
        visitar(linha - 1, coluna);
        caminhoAtual[contagem] = {linha - 1, coluna};
        resolver(linha - 1, coluna, contagem + 1);
        desvisitar(linha - 1, coluna);
    }
    if (permitido(passo, 'L') && coluna - 1 >= 0 && !grid[linha][coluna - 1]) {
        visitar(linha, coluna - 1);
        caminhoAtual[contagem] = {linha, coluna - 1};
        resolver(linha, coluna - 1, contagem + 1);
        desvisitar(linha, coluna - 1);
    }
    if (permitido(passo, 'R') && coluna + 1 < N && !grid[linha][coluna + 1]) {
        visitar(linha, coluna + 1);
        caminhoAtual[contagem] = {linha, coluna + 1};
        resolver(linha, coluna + 1, contagem + 1);
        desvisitar(linha, coluna + 1);
    }
}

int main() {
    cout << "Digite o padrao (" << N*N-1 << " caracteres com D/U/L/R/?): ";
    cin >> padrao;
    if (padrao.size() != N*N-1) {
        cerr << "Padrao deve ter exatamente " << N*N-1 << " caracteres.\n";
        return 1;
    }

    // Inicializa grau[r][c] com o número de vizinhos válidos de cada célula
    // no tabuleiro vazio (antes de qualquer visita).
    // Células de canto têm grau 2, de borda grau 3, internas grau 4.
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++) {
            grau[r][c] = 0;
            for (int d = 0; d < 4; d++) {
                int nr = r + DR[d], nc = c + DC[d];
                if (inBounds(nr, nc)) grau[r][c]++;
            }
        }

    // Marca [0][0] como visitada (ponto de partida) e atualiza graus vizinhos.
    visitar(0, 0);
    caminhoAtual[0] = {0, 0};

    auto inicio = chrono::high_resolution_clock::now();
    resolver(0, 0, 1);
    auto fim = chrono::high_resolution_clock::now();
    chrono::duration<double> tempo = fim - inicio;

    cout << "Caminhos encontrados: " << caminhos << "\n";
    cout << "Chamadas recursivas:  " << totalChamadas << "\n";
    cout << "Tempo de execucao:    " << tempo.count() << "s\n\n";

#ifdef PRINT_DEBUG
    // Imprime cada caminho como grid com número do passo em cada célula
    for (int idx = 0; idx < (int)todosCaminhos.size(); idx++) {
        const auto& cam = todosCaminhos[idx];
        cout << "=== Caminho " << idx + 1 << " ===\n";

        // Monta grade de ordem de visita
        int ordem[N][N] = {};
        for (int i = 0; i < N * N; i++)
            ordem[cam[i].first][cam[i].second] = i + 1;

        // Cabeçalho de colunas
        cout << "   ";
        for (int c = 0; c < N; c++) printf(" c%d", c);
        printf("\n");

        for (int r = 0; r < N; r++) {
            printf("r%d ", r);
            for (int c = 0; c < N; c++)
                printf("%3d", ordem[r][c]);
            printf("\n");
        }
        printf("\n");
    }
#endif

    return 0;
}