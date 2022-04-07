  /// <summary>
    /// 3D曲线
    /// </summary>
    public class SplineLine3DGenerator {

        //[SerializeField]
        Vector3[] _positions;
        //[SerializeField]
        int _sides=3;
        //[SerializeField]
        float _radiusOne=0.01f;
        //[SerializeField]
        float _radiusTwo=0.01f;
        //[SerializeField]
        //bool _useWorldSpace = true;
        //[SerializeField]
        bool _useTwoRadii = false;
        //private Vector3[] vertices;
        //private Mesh _mesh;
        public Vector3[] vertices {
            get;
            private set;
        }
        public Vector2[] uvs {
            get;
            private set;
        }
        public int[] triangles {
            get;
            private set;
        }
        public SplineLine3DGenerator() {

        }
        public SplineLine3DGenerator(int sides, float radiusOne,float radiusTwo,bool useTwoRadii) {
            _radiusOne = radiusOne;
            _radiusTwo=radiusTwo;
            _useTwoRadii = useTwoRadii;
            _sides =  Mathf.Max(3, sides);            
        }
        public void SetPositions(Vector3[] positions) {
            _positions = positions;
            _sides = Mathf.Max(3, _sides);
            GenerateMesh();
        }

        private void GenerateMesh() {
            if(//_mesh == null ||
                _positions == null ||
                _positions.Length <= 1) {
                //_mesh = new Mesh();
                return;
            }

            var verticesLength = _sides * _positions.Length;
            //if(vertices == null || vertices.Length != verticesLength) {
            {

                this.vertices = new Vector3[verticesLength];

                this.triangles = GenerateIndices();
                this.uvs = GenerateUVs();

                //if(verticesLength > _mesh.vertexCount) {
                //    _mesh.vertices = vertices;
                //    _mesh.triangles = indices;
                //    _mesh.uv = uvs;
                //}
                //else {
                //    _mesh.triangles = indices;
                //    _mesh.vertices = vertices;
                //    _mesh.uv = uvs;
                //}              
            }
            //}

            var currentVertIndex = 0;

            for(int i = 0; i < _positions.Length; i++) {
                var circle = CalculateCircle(i);
                foreach(var vertex in circle) {
                    vertices[currentVertIndex++] = vertex;//_useWorldSpace ? transform.InverseTransformPoint(vertex) : vertex;
                }
            }

            //_mesh.vertices = vertices;
            //_mesh.RecalculateNormals();
            //_mesh.RecalculateBounds();

            //_meshFilter.mesh = _mesh;
        }

        private Vector2[] GenerateUVs() {
            var uvs = new Vector2[_positions.Length * _sides];

            for(int segment = 0; segment < _positions.Length; segment++) {
                for(int side = 0; side < _sides; side++) {
                    var vertIndex = (segment * _sides + side);
                    var u = side / (_sides - 1f);
                    var v = segment / (_positions.Length - 1f);

                    uvs[vertIndex] = new Vector2(u, v);
                }
            }

            return uvs;
        }

        private int[] GenerateIndices() {
            // Two triangles and 3 vertices
            var indices = new int[_positions.Length * _sides * 2 * 3];

            var currentIndicesIndex = 0;
            for(int segment = 1; segment < _positions.Length; segment++) {
                for(int side = 0; side < _sides; side++) {
                    var vertIndex = (segment * _sides + side);
                    var prevVertIndex = vertIndex - _sides;

                    // Triangle one
                    indices[currentIndicesIndex++] = prevVertIndex;
                    indices[currentIndicesIndex++] = (side == _sides - 1) ? (vertIndex - (_sides - 1)) : (vertIndex + 1);
                    indices[currentIndicesIndex++] = vertIndex;


                    // Triangle two
                    indices[currentIndicesIndex++] = (side == _sides - 1) ? (prevVertIndex - (_sides - 1)) : (prevVertIndex + 1);
                    indices[currentIndicesIndex++] = (side == _sides - 1) ? (vertIndex - (_sides - 1)) : (vertIndex + 1);
                    indices[currentIndicesIndex++] = prevVertIndex;
                }
            }

            return indices;
        }

        private Vector3[] CalculateCircle(int index) {
            var dirCount = 0;
            var forward = Vector3.ZERO;

            // If not first index
            if(index > 0) {
                forward += (_positions[index] - _positions[index - 1]).NormalisedCopy;
                dirCount++;
            }

            // If not last index
            if(index < _positions.Length - 1) {
                forward += (_positions[index + 1] - _positions[index]).NormalisedCopy;
                dirCount++;
            }

            // Forward is the average of the connecting edges directions
            forward = (forward / dirCount).NormalisedCopy;
            var side = Mathf.Cross(forward, forward + new Vector3(.123564f, .34675f, .756892f)).NormalisedCopy;
            var up = Mathf.Cross(forward, side).NormalisedCopy;

            var circle = new Vector3[_sides];
            var angle = 0f;
            var angleStep = (2 * Mathf.PI) / _sides;

            var t = index / (_positions.Length - 1f);
            var radius = _useTwoRadii ? Mathf.Lerp(_radiusOne, _radiusTwo, t) : _radiusOne;

            for(int i = 0; i < _sides; i++) {
                var x = Mathf.Cos(angle);
                var y = Mathf.Sin(angle);

                circle[i] = _positions[index] + side * x * radius + up * y * radius;

                angle += angleStep;
            }

            return circle;
        }

        ////lineRender
        ///// <summary>
        ///// 细分
        ///// </summary>
        ////[Min(1)]
        //public int subdivisions = 3;
        ////[Min(0)]
        //public int segments = 8;
        //public Vector3[] positions;
        ////[Min(0)]
        //public float startWidth = 0.05f;
        ////[Min(0)]
        //public float endWidth = 0.05f;

        //public Vector2 uvScale = new Vector2(1f, 1f);
        //public bool inside = false;
        //private float theta = 0f;


        ////public Vector3 GetPosition(float f) {
        ////    int a = System.Math.Max(0, System.Math.Min(positions.Length, (int)System.Math.Floor(f)));
        ////    int b = System.Math.Max(0, System.Math.Min(positions.Length, (int)System.Math.Ceiling(f)));
        ////    float t = f - a;

        ////    return Vector3.Lerp(positions[a], positions[b], t);
        ////}

        //public Vector3 GetPosition(int index) {
        //    return positions[index];
        //}

        //public void SetPositions(Vector3[] positions) {
        //    this.positions = positions;
        //}




        //public void CreateMesh(out Vector3[] verts, out Vector3[] normals, out Vector2[] uvs, out int[] tris) {
        //    //Vector3[] interpolatedPositions = Enumerable.Range(0, (positions.Length - 1) * subdivisions)
        //    //    .Select(i => ((float)i) / ((float)subdivisions))
        //    //    .Select(f => GetPosition(f))
        //    //    .Append(positions.Last())
        //    //    .ToArray();
        //    List<Vector3> interpolatedPositions = new List<Vector3>();
        //    //获取顶点插值
        //    {
        //        Mogre.SimpleSpline ss = new SimpleSpline();
        //        for(int i = 0; i < this.positions.Length; i++) {
        //            ss.AddPoint(positions[i]);
        //        }
        //        int count = (positions.Length - 1) * subdivisions;
        //        for(int i = 0; i < count; i++) {
        //            interpolatedPositions.Add(ss.Interpolate(i / (float)count));
        //        }
        //        ss.Dispose();
        //        //
        //        interpolatedPositions.Add(positions[positions.Length - 1]);
        //    }
        //    //
        //    theta = (Mathf.PI * 2) / segments;

        //    //Vector3[]
        //    verts = new Vector3[interpolatedPositions.Count * segments];
        //    //Vector2[] 
        //    uvs = new Vector2[verts.Length];
        //    //Vector3[] 
        //    normals = new Vector3[verts.Length];
        //    //int[] 
        //    tris = new int[2 * 3 * verts.Length];

        //    for(int i = 0; i < interpolatedPositions.Count; i++) {
        //        float dia = Mathf.Lerp(startWidth, endWidth, (float)i / interpolatedPositions.Count);

        //        Vector3 localForward = GetVertexFwd(interpolatedPositions, i);
        //        Vector3 localUp = Mathf.Cross(localForward, Vector3.UNIT_Y);
        //        Vector3 localRight = Mathf.Cross(localForward, localUp);

        //        for(int j = 0; j < segments; ++j) {
        //            float t = theta * j;
        //            Vector3 vert = interpolatedPositions[i] + (Mathf.Sin(t) * localUp * dia) + (Mathf.Cos(t) * localRight * dia);
        //            int x = i * segments + j;
        //            verts[x] = vert;
        //            uvs[x] = uvScale * new Vector2(t / (Mathf.PI * 2), ((float)i * positions.Length) / (float)subdivisions);
        //            normals[x] = (vert - interpolatedPositions[i]).NormalisedCopy;//normalized;
        //            if(i >= interpolatedPositions.Count - 1)
        //                continue;

        //            if(inside)
        //                normals[x] = -normals[x];
        //            if(inside) {
        //                tris[x * 6] = x;
        //                tris[x * 6 + 1] = x + segments;
        //                tris[x * 6 + 2] = x + 1;

        //                tris[x * 6 + 3] = x;
        //                tris[x * 6 + 4] = x + segments - 1;
        //                tris[x * 6 + 5] = x + segments;
        //            }
        //            else {
        //                tris[x * 6] = x + 1;
        //                tris[x * 6 + 1] = x + segments;
        //                tris[x * 6 + 2] = x;

        //                tris[x * 6 + 3] = x + segments;
        //                tris[x * 6 + 4] = x + segments - 1;
        //                tris[x * 6 + 5] = x;
        //            }
        //        }
        //    }
        //    //mesh.vertices = verts;
        //    //mesh.uv = uvs;
        //    //mesh.normals = normals;
        //    //mesh.SetTriangles(tris, 0);
        //    //mesh.RecalculateBounds();
        //    //return mesh;
        //}

        //private Vector3 GetVertexFwd(List<Vector3> positions, int i) {
        //    Vector3 lastPosition;
        //    Vector3 thisPosition;
        //    if(i <= 0) {
        //        lastPosition = positions[i];
        //    }
        //    else {
        //        lastPosition = positions[i - 1];
        //    }
        //    if(i < positions.Count - 1) {
        //        thisPosition = positions[i + 1];
        //    }
        //    else {
        //        thisPosition = positions[i];
        //    }
        //    return (lastPosition - thisPosition).NormalisedCopy;//normalized;
        //}


         void test() {
            //MeshProcedural.TubeRenderer tr = new MeshProcedural.TubeRenderer();
            //tr.SetPositions(interpolatePos.ToArray());
            //Vector3[] verts;
            //Vector3[] normals;
            //Vector2[] uvs;
            //int[] tris;
            ////tr.CreateMesh(out verts,out normals,out uvs,out tris);
            //verts = tr.vertices;
            //uvs = tr.uvs;
            //tris = tr.triangles;
            //for(int i = 0; i < verts.Length; i++) {
            //    this.RenderMesh.Position(verts[i].x, verts[i].y, verts[i].z);
            //    this.RenderMesh.Colour(1f, 1f, 1f, 0.9f);
            //    this.RenderMesh.TextureCoord(uvs[i].x, uvs[i].y);
            //}
            //for(int i = 0; i < tris.Length; i++) {
            //    this.RenderMesh.Index((ushort)tris[i]);
            //}
        }

        class Mathf {

            public const float Deg2Rad = 0.0174532924F;
            public const float Rad2Deg = 57.2957764F;
            public const float HALF_PI = 1.57079637F;
            public const float NEG_INFINITY = float.NegativeInfinity;
            public const float PI = 3.14159274F;
            public const float POS_INFINITY = float.PositiveInfinity;
            public const float TWO_PI = 6.28318548F;
            //补充内容
            internal static float Distance(Vector3 v1, Vector3 v2) {
                return (v2 - v1).Length;
            }
            internal static Quaternion Inverse(Quaternion q) {
                return q.Inverse();
            }
            internal static void OrthoNormalize(ref Vector3 normal, ref Vector3 tangent) {
                //根据法线修正切线
                normal = normal.NormalisedCopy;
                //
                //q1 = m1 / |m1|
                //q2 = (m2 - (q1 ⋅ m2) * q1) / |m2 - (q1 ⋅ m2) * q1|         
                tangent -= Dot(normal, tangent) * normal;
                tangent = tangent.NormalisedCopy;
            }
            internal static float Dot(Vector3 v1, Vector3 v2) {
                return v1.DotProduct(v2);
            }
            internal static Vector3 Cross(Vector3 v1, Vector3 v2) {
                return v1.CrossProduct(v2);
            }
            internal static float Min(float a, float b) {
                return System.Math.Min(a, b);
            }
            internal static int Min(int a, int b) {
                return System.Math.Min(a, b);
            }
            internal static int FloorToInt(float f) {
                var fi = System.Math.Floor(f);
                return System.Convert.ToInt32(fi);
            }
            internal static int Ceil(float f) {
                var ci = System.Math.Ceiling(f);
                return System.Convert.ToInt32(f);
            }
            internal static Quaternion LookRotation(Vector3 direction, Vector3 upVector) {
                Quaternion q = new Quaternion();
                Vector3 zVec = direction;
                zVec = zVec.NormalisedCopy;
                Vector3 xVec = upVector.CrossProduct(zVec);
                if(xVec.IsZeroLength)
                    xVec = Vector3.UNIT_X;
                xVec = xVec.NormalisedCopy;
                Vector3 yVec = zVec.CrossProduct(xVec);
                yVec = yVec.NormalisedCopy;
                q.FromAxes(xVec, yVec, zVec);
                return q;
                //return tangent.GetRotationTo(normal);
            }

            internal static float Pow(float f, int p) {
                return (float)System.Math.Pow(f, p);
            }

            internal static float Round(float v) {
                return (float)System.Math.Round(v);
            }

            internal static float Cos(float rad) {
                return (float)System.Math.Cos(rad);
            }

            internal static float Sin(float t) {
                return (float)System.Math.Sin(t);
            }
            internal static float Lerp(float startWidth, float endWidth, float v) {
                return startWidth + (endWidth - startWidth) * v;
            }

            internal static int Max(int v, int _sides) {
                return System.Math.Max(v, _sides);
            }
        }
    }
