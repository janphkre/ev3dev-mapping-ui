using System;
using System.Collections.Generic;
using UnityEngine;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * See http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.373.59  *
 * Global Localization in SLAM in Bilinear Time                        *
 * by Lina M. Paz, Pedro Pinies, Jose Neira, Juan D. Tardos            *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
class PairingLocalization {

    //public const float PAIR_DRIVEN_G_RANGE = 10f;
    public const float PAIR_DRIVEN_G_STEP = 0.05f;
    public const float GRID_SAMPLING_RATE = 0.05f;
    public const float GRID_GROWTH = 0.01f;

    private float gridUncertanity = 2.0f;
    private int size;
    private int halfSize;
    private float gridOffsetX;
    private float gridOffsetY;

    //Internal class for the pair driven global localization.
    private class PairingVote {
        internal int votes;
        internal float bearing;
        internal HashSet<int> matchedLocalFeatures;
        internal HashSet<int> matchedGlobalFeatures;
        internal Dictionary<int, int> matches;

        internal PairingVote(float bearing) {
            votes = 1;
            this.bearing = bearing;
            matchedLocalFeatures = new HashSet<int>();
            matchedGlobalFeatures = new HashSet<int>();
            matches = new Dictionary<int, int>();
        }
    }

    private Dictionary<int, Dictionary<int, PairingVote>> voteMeasurements(FeatureCollection measurements, List<IFeature> globalFeatures, IEnumerable<int> prematchedFeatureIndexes) {
        var gridSamples = new Vector2[size, size];
        var votes = new Dictionary<int, Dictionary<int, PairingVote>>();
        int j = 0;
        foreach (Vector2 measurement in measurements.map) {
            var measurementRB = Geometry.ToRangeBearing(measurement, measurements.end);
            foreach (int feature in prematchedFeatureIndexes) {
                var f = ((Feature)globalFeatures[feature]).feature;
                for (float g = 0.0f; g < 360f; g += PAIR_DRIVEN_G_STEP) { //TODO: Add PAIR_DRIVEN_G_RANGE to not check the whole 360 degrees around each feature. 
                    var i = Geometry.FromRangeBearing(measurementRB.x, measurementRB.y + g);
                    var hypothesis = new Vector2(f.x - i.y, f.y - i.y);
                    int gridX = (int)((hypothesis.x - gridOffsetX) * size / GRID_SAMPLING_RATE) + halfSize;
                    int gridY = (int)((hypothesis.y - gridOffsetY) * size / GRID_SAMPLING_RATE) + halfSize;
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
                            if (votesY.matchedLocalFeatures.Contains(j) || votesY.matchedGlobalFeatures.Contains(feature))//TODO: remove eventually alongside Vote.matchedGlobalFeatures
                                throw new Exception("Either a feature or a measurement has already been used for this grid sample?");
                            votesY.votes++;
                        }
                    }
                    votesY.matchedLocalFeatures.Add(j);
                    votesY.matchedGlobalFeatures.Add(feature);
                    votesY.matches.Add(j, feature);
                }
            }
            j++;
        }
        return votes;
    }

    private PairingVote findHighestVote(Dictionary<int, Dictionary<int, PairingVote>> votes, out int maxX, out int maxY) {
        //Find the match as the grid sample with the highest votes:
        maxX = 0;
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
        return vote;
    }
    
    //Returns the global position of the local robot pose.
    internal Vector3 Match(FeatureCollection measurements, Vector3 globalRobotPose, List<IFeature> globalFeatures, IEnumerable<int> prematchedFeatureIndexes, float estimationError, out List<int> unmatchedLocalFeatures, out List<int> matchedGlobalFeatures) {
        gridOffsetX = globalRobotPose.x + measurements.end.x;
        gridOffsetY = globalRobotPose.y + measurements.end.y;
        size = (int)(((measurements.radius + estimationError) * gridUncertanity) / GRID_SAMPLING_RATE);
        halfSize = size / 2;
        var votes = voteMeasurements(measurements, globalFeatures, prematchedFeatureIndexes);
        int maxX, maxY;
        var vote = findHighestVote(votes, out maxX, out maxY);
        unmatchedLocalFeatures = new List<int>();
        matchedGlobalFeatures = new List<int>();
        for (int i = 0; i < measurements.map.Length; i++) {
            if (vote.matchedLocalFeatures.Contains(i)) matchedGlobalFeatures.Add(vote.matches[i]);
            else unmatchedLocalFeatures.Add(i);
        }
        //Transform match to proper values and return:
        return new Vector3((maxX - halfSize) * GRID_SAMPLING_RATE + gridOffsetX, (maxY - halfSize) * GRID_SAMPLING_RATE + gridOffsetY, (vote.bearing / vote.votes));
    }

    //Returns the location of the second pose in the first frame.
    internal Vector3 MatchServer(Vector3 firstPose, List<Vector2> firstFeatures, Vector3 secondPose, List<Vector2> secondFeatures, out int matchedCount) {
        //TODO: this requires a grid over the whole map in the first frame!
    }

    internal void increaseGridUncertanity() {
        gridUncertanity += GRID_GROWTH;
    }
}
