/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * See http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.373.59  *
 * Global Localization in SLAM in Bilinear Time                        *
 * by Lina M. Paz, Pedro Pinies, Jose Neira, Juan D. Tardos            *
 * Implementation by Jan Phillip Kretzschmar                           *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

class PairingLocalization {

    //public const float PAIR_DRIVEN_G_RANGE = 10f;
    public const float PAIR_DRIVEN_G_STEP = (float) (Math.PI / 180);
    public const float GRID_SAMPLING_RATE = 0.05f;
    public const float GRID_GROWTH = 0.01f;

    private float gridUncertanity = 2.0f;

    //Internal class for the pair driven global localization.
    private class PairingVote {
        internal int votes;
        internal float bearing;
        internal HashSet<int> matchedMeasurements;
        internal HashSet<int> matchedFeatures;
        internal Dictionary<int, int> matches;

        internal PairingVote(float bearing) {
            votes = 1;
            this.bearing = bearing;
            matchedMeasurements = new HashSet<int>();
            matchedFeatures = new HashSet<int>();
            matches = new Dictionary<int, int>();
        }
    }

    //Measurements is an enumerator over Vector2 objects.
    public Vector3 Match(Vector2 gridOffset, float gridSize, Vector3 measurementsPose, IEnumerator measurements, IEnumerator<Feature> features, out List<int> unmatchedMeasurements, out List<int> matchedFeatures) {
        int size = (int) (gridSize * gridUncertanity / GRID_SAMPLING_RATE);
        int halfSize = size / 2;
        var gridSamples = new Vector2[size, size];
        var votes = new Dictionary<int, Dictionary<int, PairingVote>>();
        int j = 0;
        while (measurements.MoveNext()) {
            var measurementRB = Geometry.ToRangeBearing((Vector2) measurements.Current, measurementsPose);
            while (features.MoveNext()) {
                var f = features.Current;
                for (float g = 0.0f; g < 2 * Math.PI; g += PAIR_DRIVEN_G_STEP) { //TODO: Add PAIR_DRIVEN_G_RANGE to not check the whole 360 degrees around each feature. 
                    var i = Geometry.FromRangeBearing(measurementRB.x, measurementRB.y + g);
                    var hypothesis = new Vector2(f.feature.x - i.y, f.feature.y - i.y);
                    int gridX = (int)((hypothesis.x - gridOffset.x) * size / GRID_SAMPLING_RATE) + halfSize;
                    int gridY = (int)((hypothesis.y - gridOffset.y) * size / GRID_SAMPLING_RATE) + halfSize;
                    if (gridX < 0) gridX = 0;
                    else continue; //if (gridX >= size) gridX = size - 1;
                    if (gridY < 0) gridY = 0;
                    else continue;//if (gridY >= size) gridY = size - 1;//TODO: This means that the grid sampling space is possibly too small. (Skip the vote then)
                    Dictionary<int, PairingVote> votesX;
                    PairingVote votesY;
                    if (!votes.TryGetValue(gridX, out votesX)) {
                        votesX = new Dictionary<int, PairingVote>();
                        votes.Add(gridX, votesX);
                        votesY = new PairingVote(g);
                        votesX[gridY] = votesY;
                    } else {
                        if (!votesX.TryGetValue(gridY, out votesY)) {
                            votesY = new PairingVote(g);
                            votesX[gridY] = votesY;

                        } else {
                            if (votesY.matchedMeasurements.Contains(j) || votesY.matchedFeatures.Contains(f.index))
                                throw new Exception("Either a feature or a measurement has already been used for this grid sample?");
                            votesY.votes++;
                        }
                    }
                    votesY.matchedMeasurements.Add(j);
                    votesY.matchedFeatures.Add(f.index);
                    votesY.matches.Add(j, f.index);
                }
            }
            features.Reset();
            j++;
        }
        features.Dispose();
        int maxX = 0,
            maxY = 0;
        PairingVote vote = new PairingVote(float.NaN);
        vote.votes = 0;
        foreach (KeyValuePair<int, Dictionary<int, PairingVote>> pairX in votes) {
            foreach (KeyValuePair<int, PairingVote> pairY in pairX.Value) {
                if (vote.votes < pairY.Value.votes) {
                    maxX = pairX.Key;
                    maxY = pairY.Key;
                    vote = pairY.Value;
                }
            }
        }
        if (vote.votes == 0) throw new Exception("An empty prematchedFeatures list or an empty localMap was provided.");
        unmatchedMeasurements = new List<int>();
        var matchedFeat = new List<int>();
        for (int i = 0; i < j; i++) {
            if (vote.matchedMeasurements.Contains(i)) matchedFeat.Add(vote.matches[i]);
            else unmatchedMeasurements.Add(i);
        }
        matchedFeatures = matchedFeat;
        //Transform match to proper values and return:
        return new Vector3((maxX - halfSize) * GRID_SAMPLING_RATE + gridOffset.x, (maxY - halfSize) * GRID_SAMPLING_RATE + gridOffset.y, (vote.bearing / vote.votes));
    }

    internal void increaseGridUncertanity() {
        gridUncertanity += GRID_GROWTH;
    }
}
