using System.Collections.Generic;
using UnityEngine;

namespace ev3devMapping.Society {

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Discrete Ant Colony Optimization                                        *
 * Artificial Intelligence for Humans Volume 2: Nature-Inspired Algorithms *
 * by Jeff Heaton                                                          *
 * Implementation by Jan Phillip Kretzschmar                               *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
class AntColonyOptimization {

    public const int ANT_COUNT = 30;
    public const float ALPHA = 1.0f;
    public const float BETA = 5.0f;
    public const float EVAPORATION = 0.5f;
    public const float Q = 500f;
    public const float INITIAL_PHEROMONE = 1.0f;
    public const float PR = 0.01f;

    private List<GraphNode> nodes;
    private DefaultedSparseFloatMatrix scores;
    private int start;
    private int target;

    private DefaultedSparseFloatMatrix pheromones;
    private float[] probabilities;
    private Ant[] ants;

    private class Ant {

        private float score = 0.0f;
        private LinkedList<int> path = new LinkedList<int>();
        private HashSet<int> visited = new HashSet<int>();

        public void Clear() {
            score = 0.0f;
            path.Clear();
            visited.Clear();
        }

        public void Add(int i, float scoreAddition) {
            path.AddLast(i);
            visited.Add(i);
            score += scoreAddition;
        }

        public void RemoveLast(float scoreSubstraction) {
            path.RemoveLast();
        }
        public bool Visited(int i) {
            return visited.Contains(i);
        }

        public int Current {
            get { return path.Last.Value; }
        }

        public int Previous {
            get { return path.Last.Previous.Value; }
        }

        public float Score {
            get { return Score; }
        }

        public IEnumerator<int> GetPath() {
            return path.GetEnumerator();
        }
    }

    public AntColonyOptimization(List<GraphNode> nodes, int start, int target) {
        this.nodes = nodes;
        scores = new DefaultedSparseFloatMatrix(float.NaN);
        pheromones = new DefaultedSparseFloatMatrix(float.NaN);
        for (int i = 0; i < nodes.Count; i++) {
            var connectedNodes = nodes[i].Connected;
            foreach (int c in connectedNodes) {
                scores[i, c] = Geometry.EuclideanDistance(nodes[i], nodes[c]);
                pheromones[i, c] = INITIAL_PHEROMONE;
            }
        }
        this.start = start;
        this.target = target;

        pheromones = new DefaultedSparseFloatMatrix(0.0f);
        pheromones.Enlarge(nodes.Count);
        probabilities = new float[nodes.Count];
        ants = new Ant[ANT_COUNT];
        for (int i = 0; i < ANT_COUNT; i++) { ants[i] = new Ant(); }
    }

    //Creates and returns a path along the highest pheromone trail.
    public LinkedList<int> GetPath() {
        var result = new LinkedList<int>();
        int i = start;
        while(i != target) {
            int max = -1;
            float maxP = 0.0f;
            foreach(int c in nodes[i].Connected) {
                float p = pheromones[i, c];
                if (p > maxP) {
                    maxP = p;
                    max = c;
                }
            }
            //if (maxP == 0.0f) throw new Exception("No Path found!");
            result.AddLast(max);
            i = max;
        }
        return result;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Listing 7.3 + Listing 7.4 + Listing 7.5:                                                            *
     * Performs one iteration of the algorithm. This consists of marching the ends and pheromone updating. *
     * Returns the smallest score.                                                                         *
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    public float Iteration() {
        //Reset the ants:
        for (int i = 0; i < ANT_COUNT; i++) {
            ants[i].Clear();
            ants[i].Add(start, 0.0f);
        }
        //March all ants for one iteration:
        for (int i = 0; i < ANT_COUNT; i++) {
            while (ants[i].Current != target) {
                int j = chooseNextStep(i);
                ants[i].Add(j, Geometry.EuclideanDistance(nodes[ants[i].Current], nodes[j]));
            }
        }
        //Pheromone Evaporation:
        for (int i = 0; i < nodes.Count; i++) {
            var connectedNodes = nodes[i].Connected;
            foreach (int c in connectedNodes) {
                pheromones[i, c] = pheromones[i, c] * EVAPORATION;
            }
        }
        //Pheromone Update:
        float min = float.MaxValue;
        for (int i = 0; i < ANT_COUNT; i++) {
            float d = Q / ants[i].Score;
            var p = ants[i].GetPath();
            p.MoveNext();
            int previous = p.Current;
            while(p.MoveNext()) {
                pheromones[previous, p.Current] += d;
                previous = p.Current;
            }
            p.Dispose();
            if (ants[i].Score < min) min = ants[i].Score;
        }
        return min;
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
     * Listing 7.1 + Listing 7.2:                                                                      *
     * Chooses a step from the weighted probabilities providied by 7.1 or a random step with PR chance.*
     * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
    private int chooseNextStep(int ant) {
        int i = ants[ant].Current;
        var connected = nodes[i].Connected;
        //Did we reach a dead end:
        int v = 0;
        foreach (int c in connected) {
            if (ants[ant].Visited(c)) v++;
        }
        if (v == connected.Count) {
            ants[ant].RemoveLast(Geometry.EuclideanDistance(nodes[i], nodes[ants[ant].Previous]));
            return chooseNextStep(ant); //Try again next time.
        }

        if (Random.Range(0.0f, 1.0f) < PR) {
            //Select a random node:
            int c;
            do {
                c = connected[Random.Range(0, connected.Count)];
            } while (ants[ant].Visited(c));
            return c;
        } else {
            //Last visited node:
            
            //Denominator:
            float d = 0f;
            var p = pheromones.GetColumn(i);
            while (p.MoveNext()) {
                if (!ants[ant].Visited(p.Current.Key)) {
                    d = d + Mathf.Pow(p.Current.Value, ALPHA) * Mathf.Pow(scores[i, p.Current.Key], BETA);
                }
            }
            //individual probabilities:
            p.Reset();
            while (p.MoveNext()) {
                if (ants[ant].Visited(p.Current.Key)) {
                    probabilities[p.Current.Key] = 0f;
                } else {
                    probabilities[p.Current.Key] = Mathf.Pow(p.Current.Value, ALPHA) * Mathf.Pow(scores[i, p.Current.Key], BETA) / d;
                }
            }
            //Sum up the probabilites until they exceed r:
            p.Reset();
            float r = Random.Range(0.0f, 1.0f),
                  sum = 0.0f;
            i = -1;
            while (p.MoveNext()) {
                i = p.Current.Key;
                sum = sum + probabilities[i];
                if (sum > r) break;
            }
            p.Dispose();
            return i;
        }
    }
}
}
