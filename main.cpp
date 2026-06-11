#include <iostream>
#include <chrono>
#include <string>
#include <vector>
using namespace std;

// descomente para ver cada poda com o estado do grid
//!!!ATENÇÃO: pode gerar muita saída no console!!!
//#define PRINT_DEBUG

const int N = 7; // tabuleiro N x N

// true se a célula (r,c) já foi visitada
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

// vizinhos não visitados de cada célula, atualizado incrementalmente
int grau[N][N];

// células com grau 1 (beco) e grau 0 (ilhada), excluindo o destino [6][0]
int global_em_beco = 0;
int global_isolados = 0;

int  caminhos = 0;
long long totalChamadas = 0;
string padrao; // 48 chars D/U/L/R/?

//auxiliares globais para salvar os caminhos verdadeiros
pair<int,int> caminhoAtual[N * N];
vector<vector<pair<int,int>>> todosCaminhos;

// baixo, cima, direita, esquerda
const int DR[] = {1, -1, 0, 0};
const int DC[] = {0, 0, 1, -1};

//!!!As funções abaixo foram marcadas como inline para melhorar a performance!!!

//funções inline para verificar se é o destino ou se está dentro do grid N x N
inline bool isDest(int r, int c)   { return r == N - 1 && c == 0; }
inline bool inBounds(int r, int c) { return r >= 0 && r < N && c >= 0 && c < N; }

// '?' aceita qualquer direção
inline bool permitido(int passo, char dir) {
    return padrao[passo] == '?' || padrao[passo] == dir;
}

// BFS nas células não visitadas a partir de (r0,c0); retorna quantas são alcançáveis
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

// marca (r,c) como visitada e atualiza grau dos vizinhos e contadores globais
void visitar(int r, int c) {
    grid[r][c] = true;
    if (!isDest(r, c)) {
        if      (grau[r][c] == 1) global_em_beco--;
        else if (grau[r][c] == 0) global_isolados--;
    }
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (!inBounds(nr, nc) || grid[nr][nc]) continue;
        grau[nr][nc]--;
        if (!isDest(nr, nc)) {
            if      (grau[nr][nc] == 1) global_em_beco++;
            else if (grau[nr][nc] == 0) { global_em_beco--; global_isolados++; }
        }
    }
}

// desfaz visitar() para backtracking
void desvisitar(int r, int c) {
    for (int d = 0; d < 4; d++) {
        int nr = r + DR[d], nc = c + DC[d];
        if (!inBounds(nr, nc) || grid[nr][nc]) continue;
        grau[nr][nc]++;
        if (!isDest(nr, nc)) {
            if      (grau[nr][nc] == 2) global_em_beco--;
            else if (grau[nr][nc] == 1) { global_em_beco++; global_isolados--; }
        }
    }
    grid[r][c] = false;
    if (!isDest(r, c)) {
        if      (grau[r][c] == 1) global_em_beco++;
        else if (grau[r][c] == 0) global_isolados++;
    }
}

void resolver(int linha, int coluna, int contagem) {
    totalChamadas++;

    if (contagem == N * N) {
        if (isDest(linha, coluna)) {
            caminhos++;
            //adiciona o caminho completo visitado dentro do vector de todosCaminhos
            todosCaminhos.emplace_back(caminhoAtual, caminhoAtual + N * N);
        }
        return;
    }

    // chegou no destino antes de visitar tudo
    if (isDest(linha, coluna)) {
#ifdef PRINT_DEBUG
        printGrid(linha, coluna, "OPT2: destino prematuro");
#endif
        return;
    }

    // alguma célula ficou sem vizinhos livres
    if (global_isolados > 0) {
#ifdef PRINT_DEBUG
        printGrid(linha, coluna, "OPT3: célula isolada");
#endif
        return;
    }

    // becos (grau 1): se há mais do que conseguimos visitar, poda
    if (global_em_beco > 0) {
        int adj_deg1 = 0;
        for (int d = 0; d < 4; d++) {
            int nr = linha + DR[d], nc = coluna + DC[d];
            if (!inBounds(nr, nc) || grid[nr][nc] || isDest(nr, nc)) continue;
            if (grau[nr][nc] == 1) adj_deg1++;
        }
        if (global_em_beco - adj_deg1 > 0) {
#ifdef PRINT_DEBUG
            printGrid(linha, coluna, "OPT3: beco inalcançável (forcados > adj_deg1)");
#endif
            return;
        }
        if (adj_deg1 > 1) {
#ifdef PRINT_DEBUG
            printGrid(linha, coluna, "OPT3: dois becos adjacentes (adj_deg1 > 1)");
#endif
            return;
        }
    }

    // BFS do destino: se não alcança todas as células restantes, tabuleiro fragmentado
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

    auto inicio = chrono::high_resolution_clock::now();
    // inicializa grau com o número de vizinhos de cada célula no tabuleiro vazio
    for (int r = 0; r < N; r++)
        for (int c = 0; c < N; c++) {
            grau[r][c] = 0;
            for (int d = 0; d < 4; d++) {
                int nr = r + DR[d], nc = c + DC[d];
                if (inBounds(nr, nc)) grau[r][c]++;
            }
        }

    visitar(0, 0);
    caminhoAtual[0] = {0, 0};


    resolver(0, 0, 1);
    auto fim = chrono::high_resolution_clock::now();
    chrono::duration<double> tempo = fim - inicio;

    cout << "Caminhos encontrados: " << caminhos << "\n";
    cout << "Chamadas recursivas:  " << totalChamadas << "\n";
    cout << "Tempo de execucao:    " << tempo.count() << "s\n\n";

#ifdef PRINT_DEBUG
    for (int idx = 0; idx < (int)todosCaminhos.size(); idx++) {
        const auto& cam = todosCaminhos[idx];
        cout << "=== Caminho " << idx + 1 << " ===\n";

        int ordem[N][N] = {};
        for (int i = 0; i < N * N; i++)
            ordem[cam[i].first][cam[i].second] = i + 1;

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