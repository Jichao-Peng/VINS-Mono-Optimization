/*****************************************************************************
**      Stereo VO and SLAM by combining point and line segment features     **
******************************************************************************
**                                                                          **
**  Copyright(c) 2016-2018, Ruben Gomez-Ojeda, University of Malaga         **
**  Copyright(c) 2016-2018, David Zuñiga-Noël, University of Malaga         **
**  Copyright(c) 2016-2018, MAPIR group, University of Malaga               **
**                                                                          **
**  This program is free software: you can redistribute it and/or modify    **
**  it under the terms of the GNU General Public License (version 3) as     **
**  published by the Free Software Foundation.                              **
**                                                                          **
**  This program is distributed in the hope that it will be useful, but     **
**  WITHOUT ANY WARRANTY; without even the implied warranty of              **
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the            **
**  GNU General Public License for more details.                            **
**                                                                          **
**  You should have received a copy of the GNU General Public License       **
**  along with this program.  If not, see <http://www.gnu.org/licenses/>.   **
**                                                                          **
*****************************************************************************/

#pragma once

//STL
#include <list>
#include <unordered_set>
#include <utility>
#include <vector>

namespace StVO {

void getLineCoords(double x1, double y1, double x2, double y2, std::list<std::pair<int, int>> &line_coords);

struct GridWindow {
    std::pair<int, int> width, height;
};

class GridStructure {
public:

    int rows, cols;

    GridStructure(int rows, int cols);

    ~GridStructure();

    std::list<int>& at(int x, int y);

    void get(int x, int y, const GridWindow &w, std::unordered_set<int> &indices) const;

    void clear();

private:

    std::vector<std::vector<std::list<int>>> grid;
    std::list<int> out_of_bounds;
};

} // namespace StVO
