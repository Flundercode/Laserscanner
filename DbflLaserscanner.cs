using System;
using System.Collections.Generic;
using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Laserscanner
{
    public class DbflLaserscanner : GH_Component
    {
        public DbflLaserscanner()
          : base("DBFL Laserscanner",
                 "Scan",
                 "Komponente zur Vorsteuerung des Laserscanners und zum Erstellen von Hoehen- und Breitenverhaeltnis basierend auf durch Bahnplanung vorgegebene Ebenen",
                 "DBFL",
                 "Scan")
        {
        }
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("planesPath", "planesPath", "planesPath", GH_ParamAccess.list);
            pManager.AddPlaneParameter("planeMaterialDeposition", "planeMaterialDeposition", "planeMaterialDeposition", GH_ParamAccess.item);
            pManager.AddNumberParameter("heightNozzle", "heightNozzle", "heightNozzle", GH_ParamAccess.item);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("ratioWidth", "ratioWidth", "ratioWidth", GH_ParamAccess.item);
            pManager.AddNumberParameter("angleScanner", "angleScanner", "angleScanner", GH_ParamAccess.item);
            pManager.AddNumberParameter("ratioHeight", "ratioHeight", "ratioHeight", GH_ParamAccess.item);
        }
        protected override void SolveInstance(IGH_DataAccess DA)
        {
            // 01. Declare variables and assign initial data.
            List<Plane> planesBase = new List<Plane>();
            List<Plane> planesPath = new List<Plane>();
            List<Vector3d> vectorsBase = new List<Vector3d>();
            List<Vector3d> vectorsPath = new List<Vector3d>();
            List<double> lengthsPath = new List<Double>();
            List<Line> segmentsPath = new List<Line>();
            List<Line> segmentsBase = new List<Line>();
            double height = 0.0;
            double radiusScanner = 200.0;
            double heightDifferenceNozzleScanner = 50.0;
            Plane planeMaterialDeposition = new Plane();
            int currentSegment = 0;

            // 02. Retrieve input data.
            if (!DA.GetDataList(0, planesBase)) return;
            if (!DA.GetDataList(0, planesPath)) return;
            if (!DA.GetData(1, ref planeMaterialDeposition)) return;
            if (!DA.GetData(2, ref height)) return;

            // 03. Validate inputs.
            if (height < 0.0) return;

            // 04. Lift planesPath by height in z-dimension to create planesBase.
            // Lift planeMaterialDeposition by height in z-dimension to create planeNozzle.
            for (int i = 0; i < planesPath.Count; i++)
            {
                Plane planeToAdd = planesBase[i];
                planeToAdd.Origin += height * Vector3d.CrossProduct(planeToAdd.XAxis, planeToAdd.YAxis);
                planesBase[i] = planeToAdd;
            }
            Plane planeNozzle = planeMaterialDeposition;
            planeNozzle.Origin -= height * planeNozzle.ZAxis;

            // 05. Compute vectors, lengths and line segments.
            for (int i = 0; i < planesPath.Count - 1; i++)
            {
                vectorsBase.Add(new Vector3d(planesBase[i + 1].Origin - planesBase[i].Origin));
                vectorsPath.Add(new Vector3d(planesPath[i + 1].Origin - planesPath[i].Origin));
                lengthsPath.Add(new Vector3d(planesPath[i + 1].Origin - planesPath[i].Origin).Length);
                segmentsPath.Add(new Line(planesPath[i].Origin, planesPath[i + 1].Origin));
                segmentsBase.Add(new Line(planesBase[i].Origin, planesBase[i + 1].Origin));
            }

            // Create segment after last planesPath and add it and its vector and length to the lists 
            Vector3d vectorAfterLastPlanePath = new Vector3d(vectorsPath[vectorsPath.Count - 1]);
            vectorAfterLastPlanePath.Unitize();
            Line lineAfterLastPlanePath = new Line(planesPath[planesPath.Count - 1].Origin, vectorAfterLastPlanePath * (radiusScanner + 1));
            vectorsPath.Add(vectorAfterLastPlanePath);
            segmentsPath.Add(lineAfterLastPlanePath);
            lengthsPath.Add(lineAfterLastPlanePath.Length);

            // 06. Choose the segment with the smallest distance to planeMaterialDeposition.Origin as
            // the current segment
            double distancePlaneMaterialDepositionToSegments = 10000.0;
            double newDistancePlaneMaterialDepositionToSegments;
            for (int i = 0; i < segmentsPath.Count - 1; i++)
            {
                newDistancePlaneMaterialDepositionToSegments = segmentsPath[i].DistanceTo(planeMaterialDeposition.Origin, true);
                if (newDistancePlaneMaterialDepositionToSegments < distancePlaneMaterialDepositionToSegments)
                {
                    distancePlaneMaterialDepositionToSegments = newDistancePlaneMaterialDepositionToSegments;
                    currentSegment = i;
                }
            }

            // 07. Create cylinder.
            Plane cylinderBase = planeNozzle;
            cylinderBase.Origin -= cylinderBase.ZAxis * heightDifferenceNozzleScanner;
            Cylinder cylinder = new Cylinder(new Circle(cylinderBase, cylinderBase.Origin, radiusScanner), 500.0);

            // 08. Calculate the traveled distance of distancePlaneMaterialDeposition.
            double distancePlaneMaterialDeposition = new Vector3d(planeMaterialDeposition.Origin - planesPath[currentSegment].Origin).Length;
            for (int i = 0; i < currentSegment; i++)
            {
                distancePlaneMaterialDeposition += lengthsPath[i];
            }

            // 10. Calculate intersections.
            List<Point3d> intersections = new List<Point3d>();
            List<int> intersectionSegments = new List<int>();
            List<double> intersectionDistances = new List<double>();
            Line linePlaneMaterialDepositionToNextPathPlane = new Line(planeMaterialDeposition.Origin, planesPath[currentSegment + 1].Origin);

            // Intersect line segment between planeMaterialDeposition and following planesPath with
            // the cylinder and output intersection1 and intersection2
            Rhino.Geometry.Intersect.LineCylinderIntersection cylinderIntersection1 = Rhino.Geometry.Intersect.Intersection.LineCylinder(linePlaneMaterialDepositionToNextPathPlane, cylinder, out Point3d intersectionCurrentSegment1, out Point3d intersectionCurrentSegment2);
            if (cylinderIntersection1 != Rhino.Geometry.Intersect.LineCylinderIntersection.None && linePlaneMaterialDepositionToNextPathPlane.DistanceTo(intersectionCurrentSegment1, true) < 0.000001)
            {
                // Add intersection, its segment and its traveled distance to lists
                intersections.Add(intersectionCurrentSegment1);
                intersectionSegments.Add(currentSegment);
                double distanceIntersection1 = new Vector3d(intersectionCurrentSegment1 - planesPath[currentSegment].Origin).Length;
                for (int j = 0; j < currentSegment; j++)
                {
                    distanceIntersection1 += lengthsPath[j];
                }
                intersectionDistances.Add(distanceIntersection1);
            }
            if (cylinderIntersection1 != Rhino.Geometry.Intersect.LineCylinderIntersection.None && linePlaneMaterialDepositionToNextPathPlane.DistanceTo(intersectionCurrentSegment2, true) < 0.000001)
            {
                intersections.Add(intersectionCurrentSegment2);
                intersectionSegments.Add(currentSegment);
                double distanceIntersection2 = new Vector3d(intersectionCurrentSegment2 - planesPath[currentSegment].Origin).Length;
                for (int j = 0; j < currentSegment; j++)
                {
                    distanceIntersection2 += lengthsPath[j];
                }
                intersectionDistances.Add(distanceIntersection2);
            }
            for (int i = currentSegment + 1; i < segmentsPath.Count; i++)
            {
                // Intersect segmentsPath with the cylinder and output intersection1 and intersection2
                Rhino.Geometry.Intersect.LineCylinderIntersection cylinderIntersection2 = Rhino.Geometry.Intersect.Intersection.LineCylinder(segmentsPath[i], cylinder, out Point3d intersection1, out Point3d intersection2);
                // First Condition: there are intersections
                // Second Condition: the determined intersections have a small distance from the
                // path segment
                if (cylinderIntersection2 != Rhino.Geometry.Intersect.LineCylinderIntersection.None && segmentsPath[i].DistanceTo(intersection1, true) < 0.000001)
                {
                    // Add the intersection, its segment and its traveled distance to lists
                    intersections.Add(intersection1);
                    intersectionSegments.Add(i);
                    double distanceIntersection1 = new Vector3d(intersection1 - planesPath[i].Origin).Length;
                    for (int j = 0; j < i; j++)
                    {
                        distanceIntersection1 += lengthsPath[j];
                    }
                    intersectionDistances.Add(distanceIntersection1);
                }
                if (cylinderIntersection2 != Rhino.Geometry.Intersect.LineCylinderIntersection.None && segmentsPath[i].DistanceTo(intersection2, true) < 0.000001)
                {
                    intersections.Add(intersection2);
                    intersectionSegments.Add(i);
                    double distanceIntersection2 = new Vector3d(intersection2 - planesPath[i].Origin).Length;
                    for (int j = 0; j < i; j++)
                    {
                        distanceIntersection2 += lengthsPath[j];
                    }
                    intersectionDistances.Add(distanceIntersection2);
                }
            }

            // 11. Set the intersection, that has the smallest traveled distance, still greater than
            // distancePlaneMaterialDeposition, as scanPosition.
            Point3d scanPosition = planeMaterialDeposition.Origin;
            double minDistance = 1000000000.0;
            int indexPossibleIntersections = 0;
            int segmentScanner = 0;
            bool validIntersection = false;
            foreach (double distance in intersectionDistances)
            {
                if (distance < minDistance && distance >= distancePlaneMaterialDeposition)
                {
                    minDistance = distance;
                    scanPosition = intersections[indexPossibleIntersections];
                    segmentScanner = intersectionSegments[indexPossibleIntersections];
                    validIntersection = true;
                }
                indexPossibleIntersections++;
            }

            // 12. Create planeScanner.
            Plane planeScanner = planeNozzle;
            Transform projectionOntoPlaneNozzle = Transform.PlanarProjection(planeNozzle);
            Point3d planeScannerOrigin = scanPosition;
            planeScannerOrigin.Transform(projectionOntoPlaneNozzle);
            planeScanner.Origin = planeScannerOrigin;
            planeScanner.Origin -= planeScanner.ZAxis * heightDifferenceNozzleScanner;

            // 13. Calculate ratioWidth.
            // Calculate absolute value, because information about orientation is not necessary.
            // Check IF validIntersection, so the correct vectorsPath[segmentScanner] can be chosen.
            Vector3d vectorNozzleScanner = new Vector3d(planeScanner.Origin - planeNozzle.Origin);
            double angleScannerWidth;
            if (validIntersection)
            {
                angleScannerWidth = Vector3d.VectorAngle(vectorsPath[segmentScanner], vectorNozzleScanner, planeNozzle);
            }
            else
            {
                angleScannerWidth = Vector3d.VectorAngle(vectorsPath[vectorsPath.Count - 1], vectorNozzleScanner, planeNozzle);
            }
            double ratioWidth = Math.Abs(Math.Cos(angleScannerWidth));

            // 14. Calculate angle.
            double angleScanner = Vector3d.VectorAngle(vectorsBase[0], vectorNozzleScanner, planeNozzle);

            // 15. Calculate ratioHeight.
            double ratioHeight = (height + heightDifferenceNozzleScanner) / new Vector3d(planeScanner.Origin - scanPosition).Length;

            // 16. Output Data.
            DA.SetData(0, ratioWidth);
            DA.SetData(1, angleScanner);
            DA.SetData(2, ratioHeight);
        }
        protected override System.Drawing.Bitmap Icon => Properties.Resources.Resources.ghIcon3;
        public override Guid ComponentGuid => new Guid("21d6be41-0a37-4769-ac27-1cc6b2b61a1a");
    }
}
