#ifndef GRID_H
#define GRID_H

#include <vector>
#include <tuple>
#include <algorithm>
#include <iostream>
#include "./util.h"
#include "./Coordinate.h"

namespace soax {
    
class Grid
{
	private:
		std::vector<std::vector<std::pair<int, int>>> grid_data;
        std::vector<std::vector<std::tuple<int, int, int>>> tag_to_bin_list;
		int numBinX, numBinY, numBinZ;
		Coordinate gridSize;
		Coordinate binSize;
		bool periodicX, periodicY, periodicZ;
        int numEntries;
	public:
        Grid();
        Grid(Coordinate b, bool px, bool py, bool pz);
		void setGridDimensions(Coordinate newGridSize);
		void setNumBins(int newNumBinX, int newNumBinY, int newNumBinZ);
        void setBinSize(double desiredBinSize);
		void setPeriodic(bool newPeriodicX, bool newPeriodicY, bool newPeriodicZ);
		int putInGrid(std::pair<int, int> tag, Coordinate c);
		void clearGrid();
        std::tuple<int, int, int> binToGridPositions(int binIndex) const;
        int gridPositionsToBin(int cellX, int cellY, int cellZ) const;
        
		int getNumBins() const;
        
        Coordinate getGridDimensions() const;
        int largestGridDimension() const;
        
        int getBin(Coordinate c) const;
        
        void shiftFirstIndexPair(int shift);
        
        std::vector<std::pair<int, int> > getTagInBins(std::vector <int> binList) const;
        std::vector<std::pair<int, int> > getNeighboringTags(Coordinate c, int level) const;
        std::vector<std::pair<int, int> > getNeighboringTags(Coordinate c) const;
        
        int getNumEntries() const;
        
        void removeElement(int indexOne);
        
        std::tuple<int, int, int> posToGridPositions(Coordinate c) const;
        
        double getMinDistBetweenIds(std::pair<int, int> a, std::pair<int,int> b) const;
        int getGridLevelDistBetweenIds(std::pair<int, int> a, std::pair<int,int> b) const;
        void constructTagList();
        
        std::tuple<int, int, int> getGridPosFromId(int a, int b) const;
        double getMinDistBetweenBins(std::tuple<int, int, int> a_grid_pos, std::tuple<int,int,int> b_grid_pos) const;
        
        //std::vector<std::pair<int, int> > getNeighboringTagsUpTo(int snakeIndex, int vertexIndex, int level) const;
        //int getLevelBetweenPos(ConstRowVec a, ConstRowVec b) const;
        
};

}
#endif