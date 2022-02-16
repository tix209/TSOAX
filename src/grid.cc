#include "include/grid.h"

namespace soax {

Grid::Grid()
{
    gridSize = Coordinate {1.0, 1.0, 1.0};
    periodicX = false;
    periodicY = false;
    periodicZ = false;
}

Grid::Grid(Coordinate b, bool px, bool py, bool pz)
{
    gridSize = b;
    
    periodicX = px;
    periodicY = py;
    periodicZ = pz;
}

void Grid::setGridDimensions(Coordinate newGridSize)
{
	gridSize = newGridSize;
}

Coordinate Grid::getGridDimensions() const
{
    return gridSize;
}

int Grid::largestGridDimension() const
{
    /*int largest_dim = numBinX;
    
    if(gridSize.y > largest_dim)
    {
        largest_dim = numBinY;
    }
    
    if(gridSize.z > largest_dim)
    {
        largest_dim = numBinZ;
    }*/
    
    return std::max(gridSize.x, std::max(gridSize.y, gridSize.z));
}

int Grid::getNumBins() const
{
	return grid_data.size();
}

int Grid::getNumEntries() const
{
    return numEntries;
}

void Grid::setNumBins(int newNumBinX, int newNumBinY, int newNumBinZ)
{
	numBinX = newNumBinX;
	numBinY = newNumBinY;
	numBinZ = newNumBinZ;
	
	grid_data.resize(numBinX*numBinY*numBinZ);
	
	binSize.x = gridSize.x / (double)numBinX;
	binSize.y = gridSize.y / (double)numBinY;
	binSize.z = gridSize.z / (double)numBinZ;
}

// sets binSize to the closest value that is larger possible based on desiredBinSize in each dimension
void Grid::setBinSize(double desiredBinSize)
{
    int tempNumBinX = (int)floor(gridSize.x / desiredBinSize);
    int tempNumBinY = (int)floor(gridSize.y / desiredBinSize);
    int tempNumBinZ = (int)floor(gridSize.z / desiredBinSize);
    
    
    setNumBins(tempNumBinX, tempNumBinY, tempNumBinZ);
}

void Grid::setPeriodic(bool newPeriodicX, bool newPeriodicY, bool newPeriodicZ)
{
	periodicX = newPeriodicX;
	periodicY = newPeriodicY;
	periodicZ = newPeriodicZ;
}

int Grid::putInGrid(std::pair<int, int> tag, Coordinate c)
{
	int binx = (int)(c.x / (double)binSize.x);
	int biny = (int)(c.y / (double)binSize.y);
	int binz = (int)(c.z / (double)binSize.z);    
    
    
    if(binx < 0)
    {
      binx = 0;
    }
    else if(binx >= numBinX)
    {
      binx = numBinX - 1;
    }

    if(biny < 0)
    {
      biny = 0;
    }
    else if(biny >= numBinY)
    {
      biny = numBinY - 1;
    }

    if(binz < 0)
    {
      binz = 0;
    }
    else if(binz >= numBinZ)
    {
      binz = numBinZ - 1;
    }
    
	int bin = (binx*numBinY*numBinZ) + biny*numBinZ + binz;
	
    grid_data[bin].push_back(tag);
    
    numEntries++;
    
    return bin;
}

void Grid::clearGrid()
{
	for(int i = 0; i < grid_data.size(); i++)
	{
		grid_data[i].clear();
	}
    
    for(int i = 0; i < tag_to_bin_list.size(); i++)
    {
        tag_to_bin_list[i].clear();
    }
    
    numEntries = 0;
}

int Grid::getBin(Coordinate c) const
{
    int binx = (int)(c.x / (double)binSize.x);
	int biny = (int)(c.y / (double)binSize.y);
	int binz = (int)(c.z / (double)binSize.z);
	
	if(binx < 0)
    {
      binx = 0;
    }
    else if(binx >= numBinX)
    {
      binx = numBinX - 1;
    }

    if(biny < 0)
    {
      biny = 0;
    }
    else if(biny >= numBinY)
    {
      biny = numBinY - 1;
    }
    
    if(binz < 0)
    {
      binz = 0;
    }
    else if(binz >= numBinZ)
    {
      binz = numBinZ - 1;
    }
	
	int bin = (binx*numBinY*numBinZ) + biny*numBinZ + binz;
	
    return bin;
}

std::tuple<int, int, int> Grid::binToGridPositions(int binIndex) const
{
    int gridX = (int)(binIndex / (double)numBinY / (double)numBinZ);
	binIndex = binIndex - gridX * numBinY * numBinZ;
	int gridY = (int)(binIndex / (double)numBinZ);
	binIndex = binIndex - gridY * numBinZ;
	int gridZ = binIndex;
    
    return std::make_tuple(gridX, gridY, gridZ);
}

std::tuple<int, int, int> Grid::posToGridPositions(Coordinate c) const
{
    int binx = (int)(c.x / (double)binSize.x);
	int biny = (int)(c.y / (double)binSize.y);
	int binz = (int)(c.z / (double)binSize.z);
	
	if(binx < 0)
    {
      binx = 0;
    }
    else if(binx >= numBinX)
    {
      binx = numBinX - 1;
    }

    if(biny < 0)
    {
      biny = 0;
    }
    else if(biny >= numBinY)
    {
      biny = numBinY - 1;
    }
    
    if(binz < 0)
    {
      binz = 0;
    }
    else if(binz >= numBinZ)
    {
      binz = numBinZ - 1;
    }
    
    return std::make_tuple(binx, biny, binz);
}

// gets the bins of a and b and then finds the max level between these two bins
/*int Grid::getLevelBetweenPos(ConstRowVec a, ConstRowVec b) const
{
    int binx_a = (int)(a.x / (double)binSize.x);
	int biny_a = (int)(a.y / (double)binSize.y);
	int binz_a = (int)(a.z / (double)binSize.z);
	
	if(binx_a < 0)
    {
      binx_a = 0;
    }
    else if(binx_a >= numBinX)
    {
      binx_a = numBinX - 1;
    }

    if(biny_a < 0)
    {
      biny_a = 0;
    }
    else if(biny_a >= numBinY)
    {
      biny_a = numBinY - 1;
    }
    
    if(binz_a < 0)
    {
      binz_a = 0;
    }
    else if(binz_a >= numBinZ)
    {
      binz_a = numBinZ - 1;
    }
    
    
    
    int binx_b = (int)(b.x / (double)binSize.x);
	int biny_b = (int)(b.y / (double)binSize.y);
	int binz_b = (int)(b.z / (double)binSize.z);
	
	if(binx_b < 0)
    {
      binx_b = 0;
    }
    else if(binx_b >= numBinX)
    {
      binx_b = numBinX - 1;
    }

    if(biny_b < 0)
    {
      biny_b = 0;
    }
    else if(biny_b >= numBinY)
    {
      biny_b = numBinY - 1;
    }
    
    if(binz_b < 0)
    {
      binz_b = 0;
    }
    else if(binz_b >= numBinZ)
    {
      binz_b = numBinZ - 1;
    }
    
    
    return std::max(fabs(binx_a-binx_b), max(fabs(biny_a-biny_b), fabs(binz_a-binz_b)));
} */
    
    

int Grid::gridPositionsToBin(int binx, int biny, int binz) const
{
    if(periodicX)
        binx = binx - numBinX*(int)floor((double)(binx)/(double)(numBinX));
    if(periodicY)
        biny = biny - numBinY*(int)floor((double)(biny)/(double)(numBinY));
    if(periodicZ)
        binz = binz - numBinZ*(int)floor((double)(binz)/(double)(numBinZ));
    
    return binx*numBinY*numBinZ + biny*numBinZ + binz;
}


std::vector<std::pair<int, int> > Grid::getTagInBins(std::vector <int> binList) const
{
    std::vector <std::pair<int, int> > tagList;
    
    for(int b = 0; b < binList.size(); b++)
    {
        int bin = binList[b];
        for(int g = 0; g < grid_data[bin].size(); g++)
        {
            std::pair<int, int>  newTag = grid_data[bin][g];
            
            tagList.push_back(newTag);                                
        }
    }
        
    return tagList;
}


std::vector<std::pair<int, int> > Grid::getNeighboringTags(Coordinate c, int level) const
{
    int centerBinIndex = getBin(c);

    std::vector <std::pair<int, int> > neighbors;
    
	int gridX, gridY, gridZ;
    
    std::tie (gridX, gridY, gridZ) = binToGridPositions(centerBinIndex);

    for(int m = -level; m <= level; m++)
	{
		for(int n = -level; n <= level; n++)
		{
			for(int o = -level; o <= level; o++)
			{
                if(abs(m) == level || abs(n) == level || abs(o) == level || (level == 1 && m == 0 && n == 0 && o == 0))
                {
                    int binx = gridX + m;
                    int biny = gridY + n;
                    int binz = gridZ + o;
                    
                    int tempgrid = gridPositionsToBin(binx, biny, binz);
                    
                    // if the neighboring bin is within the grid then add the neighbors to neighbors
                    if(tempgrid < grid_data.size() && binx >= 0 && binx < numBinX && biny >= 0 && biny < numBinY && binz >= 0 && binz < numBinZ)
                    {
                        for(int g = 0; g < grid_data[tempgrid].size(); g++)
                        {
                            std::pair<int, int>  newTag = grid_data[tempgrid][g];
                            
                            //if(newTag.first  != particleTag)
                            neighbors.push_back(newTag);
                        }
                    }
                }
			}
		}
	}
    
    return neighbors;
}



std::vector<std::pair<int, int> > Grid::getNeighboringTags(Coordinate c) const
{
    int centerBinIndex = getBin(c);

    std::vector <std::pair<int, int> > neighbors;
    
	int gridX, gridY, gridZ;
    
    std::tie (gridX, gridY, gridZ) = binToGridPositions(centerBinIndex);

    for(int m = -1; m <= 1; m++)
	{
		for(int n = -1; n <= 1; n++)
		{
			for(int o = -1; o <= 1; o++)
			{
                int binx = gridX + m;
                int biny = gridY + n;
                int binz = gridZ + o;
                
                int tempgrid = gridPositionsToBin(binx, biny, binz);
                
                // if the neighboring bin is within the grid then add the neighbors to neighbors
                if(tempgrid < grid_data.size() && binx >= 0 && binx < numBinX && biny >= 0 && biny < numBinY && binz >= 0 && binz < numBinZ)
                {
                    for(int g = 0; g < grid_data[tempgrid].size(); g++)
                    {
                        std::pair<int, int>  newTag = grid_data[tempgrid][g];
                        
                        //if(newTag.first  != particleTag)
                        neighbors.push_back(newTag);
                    }
                }
			}
		}
	}
    
    return neighbors;
}

/*std::vector<std::pair<int, int> > Grid::getNeighboringTagsUpTo(int snakeIndex, int vertexIndex, int level) const
{
    int centerBinIndex = this->getGridPosFromId(snakeIndex, vertexIndex);

    std::vector <std::pair<int, int> > neighbors;
    
	int gridX, gridY, gridZ;
    
    std::tie (gridX, gridY, gridZ) = binToGridPositions(centerBinIndex);

    for(int m = 0; m <= level; m++)
	{
		for(int n = 0; n <= level; n++)
		{
			for(int o = 0; o <= level; o++)
			{
                int binx = gridX + m;
                int biny = gridY + n;
                int binz = gridZ + o;
                
                int tempgrid = gridPositionsToBin(binx, biny, binz);
                
                // if the neighboring bin is within the grid then add the neighbors to neighbors
                if(tempgrid < grid_data.size() && binz >= 0 && binz < numBinZ && binx >= 0 && binx < numBinX && biny >= 0 && biny < numBinY)
                {
                    for(int g = 0; g < grid_data[tempgrid].size(); g++)
                    {
                        std::pair<int, int>  newTag = grid_data[tempgrid][g];
                        
                        //if(newTag.first  != particleTag)
                        neighbors.push_back(newTag);
                    }
                }
			}
		}
	}
    
    return neighbors;
}*/

void Grid::shiftFirstIndexPair(int shift)
{
    for(int i = 0; i < grid_data.size(); i++)
    {
        for(int j = 0; j < grid_data[i].size(); j++)
        {
            grid_data[i][j].first += shift;
        }
    }
}

void Grid::removeElement(int indexOne)
{
    for(int i = 0; i < grid_data.size(); i++)
    {
        for(int j = grid_data[i].size()-1; j > -1; j--)
        {
            if(grid_data[i][j].first == indexOne)
            {
                grid_data[i].erase(grid_data[i].begin() + j);
            }
            else
            {
                grid_data[i][j].first -= 1;
            }
        }
    }
}

// creates tag_to_bin_list based on data in grid_data
// tag_to_bin_list is a 2d vector contining the bin ids of vertex verIndex on snake snakeIndex
void Grid::constructTagList()
{
    for(int i = 0; i < tag_to_bin_list.size(); i++)
    {
        tag_to_bin_list[i].clear();
    }
    
    for(int i = 0; i < grid_data.size(); i++)
    {
        std::tuple<int, int, int> tmp_gridpos_i;
        
        if(grid_data[i].size() > 0)
        {
            tmp_gridpos_i = this->binToGridPositions(i);
        }
        
        for(int j = 0; j < grid_data[i].size(); j++)
        {
            int snakeIndex = grid_data[i][j].first;
            int verIndex = grid_data[i][j].second;
            
            while(tag_to_bin_list.size() <= snakeIndex)
            {
                tag_to_bin_list.push_back(std::vector<std::tuple <int, int, int>> ());
            }
            
            while(tag_to_bin_list[snakeIndex].size() <= verIndex)
            {
                tag_to_bin_list[snakeIndex].push_back(std::make_tuple(-1, -1, -1));
            }
            
            
            tag_to_bin_list[snakeIndex][verIndex] = tmp_gridpos_i;
        }
    }
}

// gets level (maximum linear distance in one dimension) between two snakeIndex, verIndex that are in tag_to_bin_list
// needs to be called after constructTagList is called
int Grid::getGridLevelDistBetweenIds(std::pair<int, int> a, std::pair<int,int> b) const
{
    std::tuple<int,int,int> a_grid_pos = tag_to_bin_list[a.first][a.second];
    std::tuple<int,int,int> b_grid_pos = tag_to_bin_list[b.first][b.second];
    
    int x_level = abs(std::get<0>(a_grid_pos) - std::get<0>(b_grid_pos));
    int y_level = abs(std::get<1>(a_grid_pos) - std::get<1>(b_grid_pos));
    int z_level = abs(std::get<2>(a_grid_pos) - std::get<2>(b_grid_pos));
    
    int max_bin_dist = std::max(x_level, std::max(y_level, z_level));
    
    return std::max(1, max_bin_dist);
}

double Grid::getMinDistBetweenIds(std::pair<int, int> a, std::pair<int,int> b) const
{
    int currLevel = getGridLevelDistBetweenIds(a, b);
    
    return (currLevel - 1) * std::min(binSize.x, binSize.y);
}

std::tuple<int, int, int> Grid::getGridPosFromId(int a, int b) const
{
    return tag_to_bin_list[a][b];
}

double Grid::getMinDistBetweenBins(std::tuple<int, int, int> a_grid_pos, std::tuple<int,int,int> b_grid_pos) const
{
    int x_level = abs(std::get<0>(a_grid_pos) - std::get<0>(b_grid_pos));
    int y_level = abs(std::get<1>(a_grid_pos) - std::get<1>(b_grid_pos));
    int z_level = abs(std::get<2>(a_grid_pos) - std::get<2>(b_grid_pos));
    
    int max_bin_dist = std::max(x_level, std::max(y_level, z_level));
    
    max_bin_dist = std::max(1, max_bin_dist);
    
    return (max_bin_dist - 1) * std::min(binSize.x, binSize.y);
}

} // namespace soax