package OBB

import (
	"awesomeProject/collision"
	"github.com/golang/geo/r3"
	"github.com/link3d-Pack3dnode/fogleman-30-07-2018/src/fauxgl"
	"github.com/quickhull-go"
	"gonum.org/v1/gonum/mat"
	"gonum.org/v1/gonum/stat"
	"sort"
)

type Obb struct {
	//obb center
	obbCenter fauxgl.Vector
	//obb vertices
	obbVertices []fauxgl.Vector
	//half edges
	Halfedges fauxgl.Vector
	//pc axes
	PCA []fauxgl.Vector
	//translation
	Translation fauxgl.Vector
	//longest axis vector
	LongestAxisVector fauxgl.Vector
	//mean of point cloud
	CentroidPointCloud fauxgl.Vector
}


func ReadPointCloud(mesh *fauxgl.Mesh) []r3.Vector{
	var pointCloud []r3.Vector
	for _, t := range mesh.Triangles {
		pointCloud = append(pointCloud, r3.Vector(t.V1.Position), r3.Vector(t.V2.Position), r3.Vector(t.V3.Position))
	}
	return pointCloud
}


func Centroid(pointCloud []r3.Vector) fauxgl.Vector{
	//Calculate mean or centroid of point cloud
	var CentroidPointCloud fauxgl.Vector
	xData := make([]float64, len(pointCloud))
	yData := make([]float64, len(pointCloud))
	zData := make([]float64, len(pointCloud))
	for i := 0; i < len(pointCloud); i++{
		xData[i] = pointCloud[i].X
		yData[i] = pointCloud[i].Y
		zData[i] = pointCloud[i].Z
	}
	CentroidPointCloud.X = stat.Mean(xData, nil)
	CentroidPointCloud.Y = stat.Mean(yData, nil)
	CentroidPointCloud.Z = stat.Mean(zData, nil)
	return CentroidPointCloud
}


func CentroidToVertex(pointCloud []r3.Vector, CentroidPointCloud fauxgl.Vector) []fauxgl.Vector{
	// Calculate vector CV = point cloud vertex - centroid
	CentroidToVertexVectors :=  make([]fauxgl.Vector, len(pointCloud))
	for i := 0; i < len(pointCloud); i++{
		CentroidToVertexVectors[i] = fauxgl.Vector(pointCloud[i]).Sub(CentroidPointCloud)
	}
	return CentroidToVertexVectors
}


func ConvexHull3D(pointCloud []r3.Vector) *mat.Dense{

	// Calculate convex hull of the point cloud
	hull := new(quickhull.QuickHull).ConvexHull(pointCloud, true, false, 0)

	data := make([]float64, len(hull.Vertices)*3)
	for i:=0; i < len(hull.Vertices); i++{
		data[3*i+0] = hull.Vertices[i].X
		data[3*i+1] = hull.Vertices[i].Y
		data[3*i+2] = hull.Vertices[i].Z
	}

	pcInput := mat.NewDense(len(hull.Vertices), 3, data)
	return pcInput
}


func PrincipalComponentAxes(pcInput *mat.Dense) mat.Dense{
	// check if principal components exist or not
	var pcType stat.PC
	ok := pcType.PrincipalComponents(pcInput, nil)

	var pcAxes mat.Dense
	if ok {
		// calculate the principal components
		pcType.VectorsTo(&pcAxes)
	}
	if !ok {
		panic("No principal components")
	}
	return pcAxes
}

func pcaVectorForm(pcAxes mat.Dense) []fauxgl.Vector{

	var PCA []fauxgl.Vector
	var pca fauxgl.Vector
	_, c := pcAxes.Dims()
	for j := 0; j < c; j++ {
		list := mat.Col(nil, j, &pcAxes)
		pca.X = list[0]
		pca.Y = list[1]
		pca.Z = list[2]
		PCA = append(PCA, pca)
	}
	return PCA
}


func ProjectPointsOnPCAxes(pcInput *mat.Dense, pcAxes mat.Dense) mat.Dense{
	// Calculate dot product of convex hull output points and principal component axes.
	var projections mat.Dense
	projections.Mul(pcInput, &pcAxes)
	return projections
}


func OBBCalculations(projections, pcAxes mat.Dense) (Halfedges, Translation, LongestAxisVector fauxgl.Vector) {

	var Translation1 fauxgl.Vector
	var Translation2 fauxgl.Vector
	var Translation3 fauxgl.Vector
	var pcaPoints fauxgl.Vector
	var pcaMinPoints fauxgl.Vector
	var pcaMaxPoints fauxgl.Vector
	var pcaMidPoints fauxgl.Vector
	var PCA []fauxgl.Vector

	_, c := projections.Dims()
	for j := 0; j < c; j++ {

		// Calculate the projections of convex hull points on each principal axes to get projected lengths
		PCAxisLengths := mat.Col(nil, j, &projections)
		sort.Float64s(PCAxisLengths)
		min := PCAxisLengths[0]
		max := PCAxisLengths[len(PCAxisLengths)-1]

		// Calculate the min and max coordinates of projected points on each axis
		pcaCol := mat.Col(nil, j, &pcAxes)
		pcaPoints.X = pcaCol[0]
		pcaPoints.Y = pcaCol[1]
		pcaPoints.Z = pcaCol[2]
		pcaMinPoints = pcaPoints.MulScalar(min)
		pcaMaxPoints = pcaPoints.MulScalar(max)
		// Calculate the mid points of the min and max projected points on each pc axis
		pcaMidPoints = (pcaMinPoints.Add(pcaMaxPoints)).DivScalar(2)
		PCA = append(PCA, pcaPoints)

		// Halfedges are given by half of the distance between the min and max projected points on each axis
		// The translation factors in x, y and z in the translation matrix (1x3) are given by the summation of midpoints of each pc axis
		if j == 0 {
			Halfedges.X = pcaMinPoints.Distance(pcaMaxPoints) / 2
			Translation1 = pcaMidPoints
			// the vector (max point - min point) is the longest axis vector (or) the normal of splitting plane that divides the point cloud
			//into positive and negative half-planes for tree subdivision
			LongestAxisVector = pcaMaxPoints.Sub(pcaMinPoints)

		}
		if j == 1 {
			Halfedges.Y = pcaMinPoints.Distance(pcaMaxPoints) / 2
			Translation2 = pcaMidPoints
		}
		if j == 2 {
			Halfedges.Z = pcaMinPoints.Distance(pcaMaxPoints) / 2
			Translation3 = pcaMidPoints
		}
	}

	Translation = Translation1.Add(Translation2.Add(Translation3))
	return Halfedges, Translation, LongestAxisVector
}

func BoxAtOrigin(Halfedges fauxgl.Vector) []float64{
	/*
		box = [h1 h2 h3
		    -h1 h2 h3
		    h1 -h2 h3
		    h1 h2 -h3
		    -h1 -h2 h3
		    -h1 h2 -h3
		    h1 -h2 -h3
		    -h1 -h2 -h3]
	*/
	boxVertices := make([]float64, 24)

	boxVertices[0] = Halfedges.X
	boxVertices[1] = Halfedges.Y
	boxVertices[2] = Halfedges.Z

	boxVertices[3] = -Halfedges.X
	boxVertices[4] = Halfedges.Y
	boxVertices[5] = Halfedges.Z

	boxVertices[6] = Halfedges.X
	boxVertices[7] = -Halfedges.Y
	boxVertices[8] = Halfedges.Z

	boxVertices[9] = Halfedges.X
	boxVertices[10] = Halfedges.Y
	boxVertices[11] = -Halfedges.Z

	boxVertices[12] = -Halfedges.X
	boxVertices[13] = -Halfedges.Y
	boxVertices[14] = Halfedges.Z

	boxVertices[15] = -Halfedges.X
	boxVertices[16] = Halfedges.Y
	boxVertices[17] = -Halfedges.Z

	boxVertices[18] = Halfedges.X
	boxVertices[19] = -Halfedges.Y
	boxVertices[20] = -Halfedges.Z

	boxVertices[21] = -Halfedges.X
	boxVertices[22] = -Halfedges.Y
	boxVertices[23] = -Halfedges.Z

	return boxVertices
}


func BoxRotated(boxVertices []float64, pcAxes mat.Dense) mat.Dense {

	obbAtOrigin := mat.NewDense(8, 3, boxVertices)
	// Box vertices are multiplied by inverse of pc axes matrix (acts as the 3x3 rotation matrix)
	//This will align the box along the pc axes
	var pcAxesInv mat.Dense
	pcAxesInv.Inverse(&pcAxes)

	var obbPCAxes mat.Dense
	obbPCAxes.Mul(obbAtOrigin, &pcAxesInv)

	return obbPCAxes
}


func BoxTranslated(Translation fauxgl.Vector, obbPCAxes mat.Dense) (obbVertices []fauxgl.Vector, obbCenter fauxgl.Vector){
	// After rotation, the box is still at origin. The box must be translated to wherever the object is located
	//The translation matrix was calculated earlier using projected midpoints
	TData := make([]float64, 3)
	TData[0] = Translation.X
	TData[1] = Translation.Y
	TData[2] = Translation.Z

	TranslationMatrix := mat.NewDense(1, 3, TData)

	var obbTranslated mat.Dense
	var obbVertex fauxgl.Vector
	var xobb []float64
	var yobb []float64
	var zobb []float64
	r, _ := obbPCAxes.Dims()

	for i := 0; i < r; i++{

		obbTranslated.Add(obbPCAxes.RowView(i).T(), TranslationMatrix)

		obbVertex.X = obbTranslated.At(0,0)
		xobb = append(xobb, obbVertex.X)

		obbVertex.Y = obbTranslated.At(0,1)
		yobb = append(yobb, obbVertex.Y)

		obbVertex.Z = obbTranslated.At(0, 2)
		zobb = append(zobb, obbVertex.Z)

		obbVertices = append(obbVertices, obbVertex)
	}

	obbCenter.X = stat.Mean(xobb, nil)
	obbCenter.Y = stat.Mean(yobb, nil)
	obbCenter.Z = stat.Mean(zobb, nil)

	return obbVertices, obbCenter
}

type OBBTree []Obb

type OBBNode struct{
	obb Obb
	Left *OBBNode
	Right *OBBNode
}

func NewOBBTree(mesh *fauxgl.Mesh, level int, subDivideOBB bool) OBBTree {

	pointCloud := ReadPointCloud(mesh)
	root := CreateOBB(pointCloud, level)
	obbtree := make(OBBTree, 1<<uint(level+1)-1)
	root.FlattenOBBTree(obbtree, 0)
	return obbtree
}

func (obbnode *OBBNode) FlattenOBBTree(obbtree OBBTree, level int) {

	obbtree[level] = obbnode.obb
	if obbnode.Left != nil {
		obbnode.Left.FlattenOBBTree(obbtree, level*2+1)
	}
	if obbnode.Right != nil {
		obbnode.Right.FlattenOBBTree(obbtree, level*2+2)
	}
}

func CreateOBB(pointCloud []r3.Vector, level int) *OBBNode {

	var obb Obb
	var obbnode *OBBNode

	obb.CentroidPointCloud = Centroid(pointCloud)
	pcInput := ConvexHull3D(pointCloud)
	pcAxes := PrincipalComponentAxes(pcInput)
	obb.PCA = pcaVectorForm(pcAxes)
	projections := ProjectPointsOnPCAxes(pcInput, pcAxes)
	obb.Halfedges, obb.Translation, obb.LongestAxisVector = OBBCalculations(projections, pcAxes)
	boxVertices := BoxAtOrigin(obb.Halfedges)
	obbPCAxes := BoxRotated(boxVertices, pcAxes)
	obb.obbVertices, obb.obbCenter = BoxTranslated(obb.Translation, obbPCAxes)

	obbnode.obb = obb
	obbnode.Left = nil
	obbnode.Right = nil
	obbnode.OBBSplit(pointCloud, level)

	return obbnode
}

func(obbnode *OBBNode) OBBSplit(pointCloud []r3.Vector, level int) {

	if level == 0{
		return
	}
	obb := obbnode.obb
	var CentroidToVertexVectors []fauxgl.Vector
	//var pointCloud []r3.Vector
	CentroidToVertexVectors = CentroidToVertex(pointCloud, obb.CentroidPointCloud)
	InsideHalfPlanePoints, OutsideHalfPlanePoints := OBBPartition(CentroidToVertexVectors, obb.LongestAxisVector, pointCloud)
	obbnode.Left = CreateOBB(InsideHalfPlanePoints, level-1)
	obbnode.Right = CreateOBB(OutsideHalfPlanePoints, level-1)
}

func OBBPartition(CentroidToVertexVectors []fauxgl.Vector, LongestAxisVector fauxgl.Vector, pointCloud []r3.Vector) (InsideHalfPlanePoints, OutsideHalfPlanePoints []r3.Vector){
	//Logic for splitting OBB along longest axis, at the centroid of the point cloud
	// dotProduct = CentroidToVertex.LongestAxisVector
	// if dotProduct < 0, the vertex belongs inside of the half plane
	// if dotProduct > 0, the vertex belongs outside of the half plane
	var dotProducts []float64
	for i := 0; i < len(CentroidToVertexVectors); i++{
		dotProduct := CentroidToVertexVectors[i].Dot(LongestAxisVector)
		dotProducts = append(dotProducts, dotProduct)
	}

	for i := 0; i < len(pointCloud); i++{
		if dotProducts[i] < 0{
			InsideHalfPlanePoints = append(InsideHalfPlanePoints, pointCloud[i])
		}else if dotProducts[i] >= 0{
			OutsideHalfPlanePoints = append(OutsideHalfPlanePoints, pointCloud[i])
		}
	}
	return InsideHalfPlanePoints, OutsideHalfPlanePoints
}

func OBBCollisionTesting(obbs []Obb, SeparatingAxes []fauxgl.Vector) bool {

	var ProjectedVerticesA []float64
	var ProjectedVerticesB []float64
	var CrossNormals []fauxgl.Vector
	var Collisions []bool

	obbA := obbs[0]
	obbB := obbs[1]

	VerticesA := obbA.obbVertices
	VerticesB := obbB.obbVertices

	for i := 0; i < len(obbA.PCA); i++{
		for j := 0; j < len(obbB.PCA); j++{
			CrossNormals = append(CrossNormals, obbA.PCA[i].Cross(obbB.PCA[j]))
		}
	}
	// SeparatingAxes = normal axes of each face of each obb + each normal obtained from the cross product of corresponding edges from both obbs
	SeparatingAxes = obbA.PCA
	SeparatingAxes = append(SeparatingAxes, obbB.PCA...)
	SeparatingAxes = append(SeparatingAxes, CrossNormals...)

	for i := 0; i < len(SeparatingAxes); i++ {

		for j := 0; j < len(VerticesA); j++ {

			ProjectedVerticesA[j] = VerticesA[j].Dot(SeparatingAxes[i])
			ProjectedVerticesB[j] = VerticesB[j].Dot(SeparatingAxes[i])
		}
		sort.Float64s(ProjectedVerticesA)
		sort.Float64s(ProjectedVerticesB)
		obbAmin := ProjectedVerticesA[0]
		obbAmax := ProjectedVerticesA[len(ProjectedVerticesA)-1]
		obbBmin := ProjectedVerticesB[0]
		obbBmax := ProjectedVerticesB[len(ProjectedVerticesB)-1]

		collision := collision.CollisionTestingLogic(obbAmin, obbAmax, obbBmin, obbBmax)
		Collisions = append(Collisions, collision)
	}
	// If and only if the collision is true for every axis, the two obbs are colliding.
	// If there is an axis on which the CollisionTestingLogic function returns false, it means that axis is the separating axis.
	// The two obbs do not collide
	var collisionvalue bool
	collisionvalue = collision.AllSameInSlice(Collisions)
	return collisionvalue
}
