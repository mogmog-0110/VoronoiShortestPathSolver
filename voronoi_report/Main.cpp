#include <Siv3D.hpp>
// openCVのヘッダファイルはSiv3D内に存在するため、includeする必要はない。

// ある点から最も近いボロノイ点を見つける関数
int findNearestPointIndex(const Point& target, const Array<Vec2>& uniquePoints);
// Dijkstraのアルゴリズムを用いて最短経路を計算する関数
Array<int> dijkstra(int start, int end, const Array<Array<int>>& adjacencyMatrix, const Array<Vec2>& uniquePoints, const Rect& AreaRect);
// ボロノイ図を生成する関数
void generateVoronoiDiagram(Subdivision2D& subdiv, Array<VoronoiFacet>& facets, Array<Polygon>& facetPolygons, HashTable<Vec2, int32>& pointIndexMap, Array<Vec2>& uniquePoints, Array<Array<int>>& adjacencyMatrix, const Rect& AreaRect);


void Main()
{
	// シーンの設定
	constexpr Size SceneSize{ 1280, 720 };
	constexpr Rect SceneRect{ SceneSize };
	constexpr Rect AreaRect = SceneRect.stretched(-50);
	const Font font{ FontMethod::MSDF, 48 };


	Window::Resize(SceneSize);
	Scene::SetBackground(ColorF{ 0.99 });

	// ボロノイ図の生成に必要な変数の宣言
	Subdivision2D subdiv{ AreaRect };
	Array<VoronoiFacet> facets;
	Array<Polygon> facetPolygons;
	HashTable<Vec2, int32> pointIndexMap;
	Array<Vec2> uniquePoints;
	Array<Array<int>> adjacencyMatrix;

	// スタート地点とゴール地点の設定
	Point start = { 100, SceneSize.y / 2 };
	Point end = { SceneSize.x - 100, SceneSize.y / 2 };

	// ボロノイ図の生成
	generateVoronoiDiagram(subdiv, facets, facetPolygons, pointIndexMap, uniquePoints, adjacencyMatrix, AreaRect);

	// スタート地点とゴール地点から最も近いボロノイ点を見つける
	int startIndex = findNearestPointIndex(start, uniquePoints);
	int endIndex = findNearestPointIndex(end, uniquePoints);
	Array<int> path = dijkstra(startIndex, endIndex, adjacencyMatrix, uniquePoints, AreaRect);

	while (System::Update())
	{
		// ボロノイ図の再生成
		if (KeyR.down())
		{
			generateVoronoiDiagram(subdiv, facets, facetPolygons, pointIndexMap, uniquePoints, adjacencyMatrix, AreaRect);
			startIndex = findNearestPointIndex(start, uniquePoints);
			endIndex = findNearestPointIndex(end, uniquePoints);
			path = dijkstra(startIndex, endIndex, adjacencyMatrix, uniquePoints, AreaRect);
		}

		// 描画処理
		SceneRect.draw(ColorF{ 0.75 });
		for (auto&& [i, facetPolygon] : Indexed(facetPolygons))
		{
			facetPolygon.draw(HSV{ (i * 25.0), 0.65, 0.8 }).drawFrame(3, ColorF{ 0.25 });
		}

		// ボロノイ図の頂点を描画
		int i = 0;
		for (const auto& facet : facets)
		{
			Circle{ facet.center, 6 }.drawFrame(5).draw(ColorF{ 0.25 });

			// 頂点数の確認
			//font(i + 1).draw(20, Vec2{ facet.center.x -1, facet.center.y -1});
			i++;
		}

		for (const auto& point : uniquePoints)
		{
			Circle{ point, 3 }.draw(ColorF{ Palette::Ghostwhite });
		}

		Circle{ start, 10 }.draw(ColorF{ Palette::Red });
		Circle{ end, 10 }.draw(ColorF{ Palette::Blue });

		if (!path.isEmpty())
		{
			Line(start, uniquePoints[path.front()]).draw(5, ColorF{ Palette::Yellow });
			Line(end, uniquePoints[path.back()]).draw(5, ColorF{ Palette::Yellow });
			for (size_t i = 0; i < path.size() - 1; ++i)
			{
				Line(uniquePoints[path[i]], uniquePoints[path[i + 1]]).draw(5, ColorF{ Palette::Yellow });
			}
		}
	}
}

void generateVoronoiDiagram(Subdivision2D& subdiv, Array<VoronoiFacet>& facets, Array<Polygon>& facetPolygons, HashTable<Vec2, int32>& pointIndexMap, Array<Vec2>& uniquePoints, Array<Array<int>>& adjacencyMatrix, const Rect& AreaRect)
{
	subdiv = Subdivision2D{ AreaRect };
	pointIndexMap.clear();
	uniquePoints.clear();
	facets.clear();
	facetPolygons.clear();
	adjacencyMatrix.clear();

	for (int i = 0; i < 50; ++i)
	{
		subdiv.addPoint(RandomVec2(AreaRect));
	}

	subdiv.calculateVoronoiFacets(facets);
	facetPolygons = facets.map([AreaRect](const VoronoiFacet& f) {
		return Geometry2D::And(Polygon{ f.points }, AreaRect).front();
	});

	int index = 0;
	for (const auto& facet : facets)
	{
		for (const auto& point : facet.points)
		{
			if (!pointIndexMap.contains(point))
			{
				pointIndexMap[point] = index++;
				uniquePoints.push_back(point);
			}
		}
	}

	adjacencyMatrix.resize(uniquePoints.size(), Array<int>(uniquePoints.size(), 0));
	for (const auto& facet : facets)
	{
		for (size_t i = 0; i < facet.points.size(); ++i)
		{
			int currentIndex = pointIndexMap[facet.points[i]];
			int nextIndex = pointIndexMap[facet.points[(i + 1) % facet.points.size()]];
			adjacencyMatrix[currentIndex][nextIndex] = 1;
			adjacencyMatrix[nextIndex][currentIndex] = 1;
		}
	}
}


// スタートとゴール地点から最も近いボロノイ点を見つける関数
int findNearestPointIndex(const Point& target, const Array<Vec2>& uniquePoints)
{
	double minDistance = std::numeric_limits<double>::max();
	int nearestIndex = -1;

	for (int i = 0; i < uniquePoints.size(); ++i)
	{
		double distance = target.distanceFrom(uniquePoints[i]);
		if (distance < minDistance)
		{
			minDistance = distance;
			nearestIndex = i;
		}
	}

	return nearestIndex;
}

// Dijkstraのアルゴリズムを用いて最短経路を計算する関数
Array<int> dijkstra(int start, int end, const Array<Array<int>>& adjacencyMatrix, const Array<Vec2>& uniquePoints, const Rect& AreaRect)
{
	int n = adjacencyMatrix.size();
	Array<int> dist(n, std::numeric_limits<int>::max());
	Array<int> prev(n, -1);
	std::priority_queue<std::pair<int, int>, std::vector<std::pair<int, int>>, std::greater<std::pair<int, int>>> pq;

	dist[start] = 0;
	pq.push({ 0, start });

	while (!pq.empty())
	{
		auto [cost, v] = pq.top();
		pq.pop();

		if (cost > dist[v]) continue;

		for (int u = 0; u < n; ++u)
		{
			// AreaRect 内に存在する頂点のみを考慮する
			if (adjacencyMatrix[v][u] != 0 && dist[u] > dist[v] + adjacencyMatrix[v][u] && AreaRect.contains(uniquePoints[u]))
			{
				dist[u] = dist[v] + adjacencyMatrix[v][u];
				prev[u] = v;
				pq.push({ dist[u], u });
			}
		}
	}

	// 経路の復元
	Array<int> path;
	for (int at = end; at != -1; at = prev[at])
	{
		if (AreaRect.contains(uniquePoints[at]))
		{
			path.push_back(at);
		}
	}
	std::reverse(path.begin(), path.end());

	return path;
}
