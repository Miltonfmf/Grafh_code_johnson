import heapq
import sys

class Grafo:
    #inicia a classe grafo
    def __init__(self, vertices):
        self.V = vertices
        self.grafo = []
    #adiciona aresta ao grafo
    def add_aresta(self, u, v, peso):
        self.grafo.append([u, v, peso])
    #encontra a menor distancia detectando ciclos negativos
    def bellman_ford(self, origem):
        dist = [float("Inf")] * (self.V + 1)
        dist[origem] = 0
        
        for __init__ in range(self.V):
            for u, v, peso in self.grafo:
                if dist[u] != float("Inf") and dist[u] + peso < dist[v]:
                    dist[v] = dist[u] + peso

        for u, v, peso in self.grafo:
            if dist[u] != float("Inf") and dist[u] + peso < dist[v]:
                print("O grafo contém um ciclo de peso negativo")
                return None
        
        return dist
    #procura o caminho mais curto 
    def dijkstra(self, origem):
        dist = [float("Inf")] * self.V
        dist[origem] = 0
        pq = [(0, origem)]

        while pq:
            d, u = heapq.heappop(pq)
            if d > dist[u]:
                continue

            for aresta in self.grafo:
                if aresta[0] == u:
                    v = aresta[1]
                    peso = aresta[2]
                    if dist[u] + peso < dist[v]:
                        dist[v] = dist[u] + peso
                        heapq.heappush(pq, (dist[v], v))

        return dist
    #add um novo vertice e conecta ou outros com arestas peso zero
    #recalibra os pesos para eliminar valores negativos
    def johnson(self):
        novo_grafo = Grafo(self.V + 1)
        novo_grafo.grafo = self.grafo[:]
        for i in range(self.V):
            novo_grafo.add_aresta(self.V, i, 0)

        h = novo_grafo.bellman_ford(self.V)
        if h is None:
            return None

        novo_grafo.grafo = []
        for u, v, peso in self.grafo:
            novo_grafo.add_aresta(u, v, peso + h[u] - h[v])

        distancias = []
        for u in range(self.V):
            distancias.append(self.dijkstra(u))

        for u in range(self.V):
            for v in range(self.V):
                if distancias[u][v] != float("Inf"):
                    distancias[u][v] += h[v] - h[u]

        return distancias
#uso
g = Grafo(5)
g.add_aresta(0, 1, -1)
g.add_aresta(0, 2, 4)
g.add_aresta(1, 2, 3)
g.add_aresta(1, 3, 2)
g.add_aresta(1, 4, 2)
g.add_aresta(3, 2, 5)
g.add_aresta(3, 1, 1)
g.add_aresta(4, 3, -3)

distancias = g.johnson()
if distancias:
    for i in range(len(distancias)):
        print(f"Menores distâncias a partir do vértice {i}: {distancias[i]}")
