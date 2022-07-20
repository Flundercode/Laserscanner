using System;
using System.Collections.Generic;
using Grasshopper;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace Laserscanner
{
    public class DbflLaserscannerNew : GH_Component
    {
        public DbflLaserscannerNew()
          : base("DBFL Laserscanner New",
                 "Scan New",
                 "Komponente zur Vorsteuerung des Laserscanners und zum Erstellen von Hoehen- und Breitenverhaeltnis basierend auf durch Bahnplanung vorgegebene Ebenen",
                 "DBFL",
                 "Scan")
        {
        }
        protected override void RegisterInputParams(GH_Component.GH_InputParamManager pManager)
        {
            pManager.AddPlaneParameter("planesPath", "planesPath", "planesPath", GH_ParamAccess.list);
            pManager.AddPlaneParameter("planeMaterialDeposition", "planeMaterialDeposition", "planeMaterialDeposition", GH_ParamAccess.list);
            pManager.AddNumberParameter("heightNozzle", "heightNozzle", "heightNozzle", GH_ParamAccess.item);
            pManager.AddNumberParameter("radiusScanner", "radiusScanner", "radiusScanner", GH_ParamAccess.item);
            pManager.AddNumberParameter("heightDifferenceNozzleScanner", "heightDifferenceNozzleScanner", "heightDifferenceNozzleScanner", GH_ParamAccess.item);
        }
        protected override void RegisterOutputParams(GH_Component.GH_OutputParamManager pManager)
        {
            pManager.AddNumberParameter("ratioWidth", "ratioWidth", "ratioWidth", GH_ParamAccess.list);
            pManager.AddNumberParameter("angleScanner", "angleScanner", "angleScanner", GH_ParamAccess.list);
            pManager.AddNumberParameter("ratioHeight", "ratioHeight", "ratioHeight", GH_ParamAccess.list);
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
            double radiusScanner = new double(); //200.0;
            double heightDifferenceNozzleScanner = new double(); //50.0;
            List<Plane> planeMaterialDeposition = new List<Plane>();
            List<double> ratioWidth = new List<double>();
            List<double> angleScanner = new List<double>();
            List<double> ratioHeight = new List<double>();

            // 02. Retrieve input data.
            if (!DA.GetDataList(0, planesBase)) return;
            if (!DA.GetDataList(0, planesPath)) return;
            if (!DA.GetDataList(1, planeMaterialDeposition)) return;
            if (!DA.GetData(2, ref height)) return;
            if (!DA.GetData(3, ref radiusScanner)) return;
            if (!DA.GetData(4, ref heightDifferenceNozzleScanner)) return;

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

            int currentSegment = 0;

            for (int index = 0; index < planeMaterialDeposition.Count - 1; index++)
            {
                Plane planeNozzle = planeMaterialDeposition[index];
                planeNozzle.Origin -= height * planeNozzle.ZAxis;                

                // 06. Choose the segment with the smallest distance to planeMaterialDeposition.Origin as
                // the current segment
                double distancePlaneMaterialDepositionToSegments = 10000.0;
                double newDistancePlaneMaterialDepositionToSegments;
                for (int i = 0; i < segmentsPath.Count - 1; i++)
                {
                    newDistancePlaneMaterialDepositionToSegments = segmentsPath[i].DistanceTo(planeMaterialDeposition[index].Origin, true);
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
                double distancePlaneMaterialDeposition = new Vector3d(planeMaterialDeposition[index].Origin - planesPath[currentSegment].Origin).Length;
                for (int i = 0; i < currentSegment; i++)
                {
                    distancePlaneMaterialDeposition += lengthsPath[i];
                }

                // 10. Calculate intersections.
                List<Point3d> intersections = new List<Point3d>();
                List<int> intersectionSegments = new List<int>();
                List<double> intersectionDistances = new List<double>();
                Line linePlaneMaterialDepositionToNextPathPlane = new Line(planeMaterialDeposition[index].Origin, planesPath[currentSegment + 1].Origin);

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
                    // Second Condition: the determined intersections have a small distance from the path segment

                    if (cylinderIntersection2 != Rhino.Geometry.Intersect.LineCylinderIntersection.None && segmentsPath[i].DistanceTo(intersection1, true) < 0.000001)
                    {
                        // Add the intersection, its segment and its traveled distance to lists
                        intersections.Add(intersection1);
                        intersectionSegments.Add(i);
                        double distanceIntersection1 = new Vector3d(intersection1 - planesPath[i].Origin).Length; //planesPatch[] variabler index, planesPath[i] bad
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
                        double distanceIntersection2 = new Vector3d(intersection2 - planesPath[i].Origin).Length; //planesPatch[] variabler index, planesPath[i]
                        for (int j = 0; j < i; j++)
                        {
                            distanceIntersection2 += lengthsPath[j];
                        }
                        intersectionDistances.Add(distanceIntersection2);
                    }
                }

                // 11. Set the intersection, that has the smallest traveled distance, still greater than
                // distancePlaneMaterialDeposition, as scanPosition.
                Point3d scanPosition = planeMaterialDeposition[index].Origin;
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
                ratioWidth.Add(Math.Abs(Math.Cos(angleScannerWidth)));

                // 14. Calculate angle.
                angleScanner.Add(Vector3d.VectorAngle(vectorsBase[0], vectorNozzleScanner, planeNozzle));               

                // 15. Calculate ratioHeight.
                ratioHeight.Add((height + heightDifferenceNozzleScanner) / new Vector3d(planeScanner.Origin - scanPosition).Length);

                // 16. Empty temporary lists
                intersections.Clear();
                intersectionSegments.Clear();
                intersectionDistances.Clear();
            }

            // 17. Output Data.
            DA.SetDataList(0, ratioWidth);
            DA.SetDataList(1, angleScanner);
            DA.SetDataList(2, ratioHeight);
        }
        protected override System.Drawing.Bitmap Icon => Properties.Resources.Resources.ghIcon3;
        public override Guid ComponentGuid => new Guid("1234abcd-1234-abcd-5678-abc123def456");
    }
}
