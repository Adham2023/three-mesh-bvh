import{S as F,n as h,V as T,W as j,c as Y,D as $,A as K,P as Q,ao as X,b as J,M as Z,g as ee,a as te,R as C,F as q,H as oe,L as D,ap as ne,Y as ie,t as re,aq as ae}from"./ExtendedTriangle-CtXgmvQ9.js";import{G as se}from"./GLTFLoader-CZLkWWGT.js";import{F as _}from"./Pass-B3TZD4e-.js";import{O as ce}from"./OrbitControls-c3zRx6W8.js";import{g as de}from"./lil-gui.module.min-Bc0DeA9g.js";import{S as le}from"./stats.min-GTpOrGrX.js";import{G as ue}from"./GenerateMeshBVHWorker-CvqnR6RV.js";import{M as ve}from"./MeshBVHUniformStruct-D80hxNjE.js";import{c as fe,b as me,a as pe}from"./bvh_struct_definitions.glsl-SZg5BxSQ.js";import{M as xe}from"./meshopt_decoder.module-Cf1-17OU.js";import{S as he}from"./StaticGeometryGenerator-CgsOCfdS.js";import"./BufferGeometryUtils-DXOWBKgz.js";import"./_commonjsHelpers-Cpj98o6Y.js";import"./MeshBVH-gvUqwvsz.js";const ye=`

float dot2( vec3 v ) {

	return dot( v, v );

}

// https://www.shadertoy.com/view/ttfGWl
vec3 closestPointToTriangle( vec3 p, vec3 v0, vec3 v1, vec3 v2, out vec3 barycoord ) {

    vec3 v10 = v1 - v0;
    vec3 v21 = v2 - v1;
    vec3 v02 = v0 - v2;

	vec3 p0 = p - v0;
	vec3 p1 = p - v1;
	vec3 p2 = p - v2;

    vec3 nor = cross( v10, v02 );

    // method 2, in barycentric space
    vec3  q = cross( nor, p0 );
    float d = 1.0 / dot2( nor );
    float u = d * dot( q, v02 );
    float v = d * dot( q, v10 );
    float w = 1.0 - u - v;

	if( u < 0.0 ) {

		w = clamp( dot( p2, v02 ) / dot2( v02 ), 0.0, 1.0 );
		u = 0.0;
		v = 1.0 - w;

	} else if( v < 0.0 ) {

		u = clamp( dot( p0, v10 ) / dot2( v10 ), 0.0, 1.0 );
		v = 0.0;
		w = 1.0 - u;

	} else if( w < 0.0 ) {

		v = clamp( dot( p1, v21 ) / dot2( v21 ), 0.0, 1.0 );
		w = 0.0;
		u = 1.0-v;

	}

	barycoord = vec3( u, v, w );
    return u * v1 + v * v2 + w * v0;

}

float distanceToTriangles(
	// geometry info and triangle range
	sampler2D positionAttr, usampler2D indexAttr, uint offset, uint count,

	// point and cut off range
	vec3 point, float closestDistanceSquared,

	// outputs
	inout uvec4 faceIndices, inout vec3 faceNormal, inout vec3 barycoord, inout float side, inout vec3 outPoint
) {

	bool found = false;
	vec3 localBarycoord;
	for ( uint i = offset, l = offset + count; i < l; i ++ ) {

		uvec3 indices = uTexelFetch1D( indexAttr, i ).xyz;
		vec3 a = texelFetch1D( positionAttr, indices.x ).rgb;
		vec3 b = texelFetch1D( positionAttr, indices.y ).rgb;
		vec3 c = texelFetch1D( positionAttr, indices.z ).rgb;

		// get the closest point and barycoord
		vec3 closestPoint = closestPointToTriangle( point, a, b, c, localBarycoord );
		vec3 delta = point - closestPoint;
		float sqDist = dot2( delta );
		if ( sqDist < closestDistanceSquared ) {

			// set the output results
			closestDistanceSquared = sqDist;
			faceIndices = uvec4( indices.xyz, i );
			faceNormal = normalize( cross( a - b, b - c ) );
			barycoord = localBarycoord;
			outPoint = closestPoint;
			side = sign( dot( faceNormal, delta ) );

		}

	}

	return closestDistanceSquared;

}

float distanceSqToBounds( vec3 point, vec3 boundsMin, vec3 boundsMax ) {

	vec3 clampedPoint = clamp( point, boundsMin, boundsMax );
	vec3 delta = point - clampedPoint;
	return dot( delta, delta );

}

float distanceSqToBVHNodeBoundsPoint( vec3 point, sampler2D bvhBounds, uint currNodeIndex ) {

	uint cni2 = currNodeIndex * 2u;
	vec3 boundsMin = texelFetch1D( bvhBounds, cni2 ).xyz;
	vec3 boundsMax = texelFetch1D( bvhBounds, cni2 + 1u ).xyz;
	return distanceSqToBounds( point, boundsMin, boundsMax );

}

// use a macro to hide the fact that we need to expand the struct into separate fields
#define	bvhClosestPointToPoint(		bvh,		point, faceIndices, faceNormal, barycoord, side, outPoint	)	_bvhClosestPointToPoint(		bvh.position, bvh.index, bvh.bvhBounds, bvh.bvhContents,		point, faceIndices, faceNormal, barycoord, side, outPoint	)

float _bvhClosestPointToPoint(
	// bvh info
	sampler2D bvh_position, usampler2D bvh_index, sampler2D bvh_bvhBounds, usampler2D bvh_bvhContents,

	// point to check
	vec3 point,

	// output variables
	inout uvec4 faceIndices, inout vec3 faceNormal, inout vec3 barycoord,
	inout float side, inout vec3 outPoint
 ) {

	// stack needs to be twice as long as the deepest tree we expect because
	// we push both the left and right child onto the stack every traversal
	int ptr = 0;
	uint stack[ BVH_STACK_DEPTH ];
	stack[ 0 ] = 0u;

	float closestDistanceSquared = pow( 100000.0, 2.0 );
	bool found = false;
	while ( ptr > - 1 && ptr < BVH_STACK_DEPTH ) {

		uint currNodeIndex = stack[ ptr ];
		ptr --;

		// check if we intersect the current bounds
		float boundsHitDistance = distanceSqToBVHNodeBoundsPoint( point, bvh_bvhBounds, currNodeIndex );
		if ( boundsHitDistance > closestDistanceSquared ) {

			continue;

		}

		uvec2 boundsInfo = uTexelFetch1D( bvh_bvhContents, currNodeIndex ).xy;
		bool isLeaf = bool( boundsInfo.x & 0xffff0000u );
		if ( isLeaf ) {

			uint count = boundsInfo.x & 0x0000ffffu;
			uint offset = boundsInfo.y;
			closestDistanceSquared = distanceToTriangles(
				bvh_position, bvh_index, offset, count, point, closestDistanceSquared,

				// outputs
				faceIndices, faceNormal, barycoord, side, outPoint
			);

		} else {

			uint leftIndex = currNodeIndex + 1u;
			uint splitAxis = boundsInfo.x & 0x0000ffffu;
			uint rightIndex = boundsInfo.y;
			bool leftToRight = distanceSqToBVHNodeBoundsPoint( point, bvh_bvhBounds, leftIndex ) < distanceSqToBVHNodeBoundsPoint( point, bvh_bvhBounds, rightIndex );//rayDirection[ splitAxis ] >= 0.0;
			uint c1 = leftToRight ? leftIndex : rightIndex;
			uint c2 = leftToRight ? rightIndex : leftIndex;

			// set c2 in the stack so we traverse it later. We need to keep track of a pointer in
			// the stack while we traverse. The second pointer added is the one that will be
			// traversed first
			ptr ++;
			stack[ ptr ] = c2;
			ptr ++;
			stack[ ptr ] = c1;

		}

	}

	return sqrt( closestDistanceSquared );

}
`;class ge extends F{constructor(o){super({defines:{USE_SHADER_RAYCAST:window.location.hash.includes("USE_SHADER_RAYCAST")?1:0},uniforms:{matrix:{value:new h},zValue:{value:0},bvh:{value:new ve}},vertexShader:`

				varying vec2 vUv;

				void main() {

					vUv = uv;
					gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );

				}

			`,fragmentShader:`

				precision highp isampler2D;
				precision highp usampler2D;

				${fe}
				${me}
				${pe}
				${ye}

				varying vec2 vUv;

				uniform BVH bvh;
				uniform float zValue;
				uniform mat4 matrix;

				void main() {

					// compute the point in space to check
					vec3 point = vec3( vUv, zValue );
					point -= vec3( 0.5 );
					point = ( matrix * vec4( point, 1.0 ) ).xyz;

					// retrieve the distance and other values
					uvec4 faceIndices;
					vec3 faceNormal;
					vec3 barycoord;
					float side;
					float rayDist;
					vec3 outPoint;
					float dist = bvhClosestPointToPoint( bvh, point.xyz, faceIndices, faceNormal, barycoord, side, outPoint );

					// This currently causes issues on some devices when rendering to 3d textures and texture arrays
					#if USE_SHADER_RAYCAST

					side = 1.0;
					bvhIntersectFirstHit( bvh, point.xyz, vec3( 0.0, 0.0, 1.0 ), faceIndices, faceNormal, barycoord, side, rayDist );

					#endif

					// if the triangle side is the back then it must be on the inside and the value negative
					gl_FragColor = vec4( side * dist, 0, 0, 0 );

				}

			`}),this.setValues(o)}}class be extends F{constructor(o){super({defines:{DISPLAY_GRID:0},uniforms:{sdfTex:{value:null},layer:{value:0},layers:{value:0}},vertexShader:`

				varying vec2 vUv;

				void main() {

					vUv = uv;
					gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );

				}

			`,fragmentShader:`
				precision highp sampler3D;

				varying vec2 vUv;
				uniform sampler3D sdfTex;
				uniform float layer;
				uniform float layers;

				void main() {

					#if DISPLAY_GRID

					float dim = ceil( sqrt( layers ) );
					vec2 cell = floor( vUv * dim );
					vec2 frac = vUv * dim - cell;
					float zLayer = ( cell.y * dim + cell.x ) / ( dim * dim );

					float dist = texture( sdfTex, vec3( frac, zLayer ) ).r;
					gl_FragColor.rgb = dist > 0.0 ? vec3( 0, dist, 0 ) : vec3( - dist, 0, 0 );
					gl_FragColor.a = 1.0;

					#else

					float dist = texture( sdfTex, vec3( vUv, layer ) ).r;
					gl_FragColor.rgb = dist > 0.0 ? vec3( 0, dist, 0 ) : vec3( - dist, 0, 0 );
					gl_FragColor.a = 1.0;

					#endif

					#include <colorspace_fragment>

				}
			`}),this.setValues(o)}}class we extends F{constructor(o){super({defines:{MAX_STEPS:500,SURFACE_EPSILON:.001},uniforms:{surface:{value:0},sdfTex:{value:null},normalStep:{value:new T},projectionInverse:{value:new h},sdfTransformInverse:{value:new h}},vertexShader:`

				varying vec2 vUv;

				void main() {

					vUv = uv;
					gl_Position = projectionMatrix * modelViewMatrix * vec4( position, 1.0 );

				}

			`,fragmentShader:`
				precision highp sampler3D;

				varying vec2 vUv;

				uniform float surface;
				uniform sampler3D sdfTex;
				uniform vec3 normalStep;
				uniform mat4 projectionInverse;
				uniform mat4 sdfTransformInverse;

				#include <common>

				// distance to box bounds
				vec2 rayBoxDist( vec3 boundsMin, vec3 boundsMax, vec3 rayOrigin, vec3 rayDir ) {

					vec3 t0 = ( boundsMin - rayOrigin ) / rayDir;
					vec3 t1 = ( boundsMax - rayOrigin ) / rayDir;
					vec3 tmin = min( t0, t1 );
					vec3 tmax = max( t0, t1 );

					float distA = max( max( tmin.x, tmin.y ), tmin.z );
					float distB = min( tmax.x, min( tmax.y, tmax.z ) );

					float distToBox = max( 0.0, distA );
					float distInsideBox = max( 0.0, distB - distToBox );
					return vec2( distToBox, distInsideBox );

				}

				void main() {

					// get the inverse of the sdf box transform
					mat4 sdfTransform = inverse( sdfTransformInverse );

					// convert the uv to clip space for ray transformation
					vec2 clipSpace = 2.0 * vUv - vec2( 1.0 );

					// get world ray direction
					vec3 rayOrigin = vec3( 0.0 );
					vec4 homogenousDirection = projectionInverse * vec4( clipSpace, - 1.0, 1.0 );
					vec3 rayDirection = normalize( homogenousDirection.xyz / homogenousDirection.w );

					// transform ray into local coordinates of sdf bounds
					vec3 sdfRayOrigin = ( sdfTransformInverse * vec4( rayOrigin, 1.0 ) ).xyz;
					vec3 sdfRayDirection = normalize( ( sdfTransformInverse * vec4( rayDirection, 0.0 ) ).xyz );

					// find whether our ray hits the box bounds in the local box space
					vec2 boxIntersectionInfo = rayBoxDist( vec3( - 0.5 ), vec3( 0.5 ), sdfRayOrigin, sdfRayDirection );
					float distToBox = boxIntersectionInfo.x;
					float distInsideBox = boxIntersectionInfo.y;
					bool intersectsBox = distInsideBox > 0.0;

					gl_FragColor = vec4( 0.0 );
					if ( intersectsBox ) {

						// find the surface point in world space
						bool intersectsSurface = false;
						vec4 localPoint = vec4( sdfRayOrigin + sdfRayDirection * ( distToBox + 1e-5 ), 1.0 );
						vec4 point = sdfTransform * localPoint;

						// ray march
						for ( int i = 0; i < MAX_STEPS; i ++ ) {

							// sdf box extends from - 0.5 to 0.5
							// transform into the local bounds space [ 0, 1 ] and check if we're inside the bounds
							vec3 uv = ( sdfTransformInverse * point ).xyz + vec3( 0.5 );
							if ( uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0 || uv.z < 0.0 || uv.z > 1.0 ) {

								break;

							}

							// get the distance to surface and exit the loop if we're close to the surface
							float distanceToSurface = texture2D( sdfTex, uv ).r - surface;
							if ( distanceToSurface < SURFACE_EPSILON ) {

								intersectsSurface = true;
								break;

							}

							// step the ray
							point.xyz += rayDirection * abs( distanceToSurface );

						}

						// find the surface normal
						if ( intersectsSurface ) {

							// compute the surface normal
							vec3 uv = ( sdfTransformInverse * point ).xyz + vec3( 0.5 );
							float dx = texture( sdfTex, uv + vec3( normalStep.x, 0.0, 0.0 ) ).r - texture( sdfTex, uv - vec3( normalStep.x, 0.0, 0.0 ) ).r;
							float dy = texture( sdfTex, uv + vec3( 0.0, normalStep.y, 0.0 ) ).r - texture( sdfTex, uv - vec3( 0.0, normalStep.y, 0.0 ) ).r;
							float dz = texture( sdfTex, uv + vec3( 0.0, 0.0, normalStep.z ) ).r - texture( sdfTex, uv - vec3( 0.0, 0.0, normalStep.z ) ).r;
							vec3 normal = normalize( vec3( dx, dy, dz ) );

							// compute some basic lighting effects
							vec3 lightDirection = normalize( vec3( 1.0 ) );
							float lightIntensity =
								saturate( dot( normal, lightDirection ) ) +
								saturate( dot( normal, - lightDirection ) ) * 0.05 +
								0.1;
							gl_FragColor.rgb = vec3( lightIntensity );
							gl_FragColor.a = 1.0;

						}

					}

					#include <colorspace_fragment>

				}
			`}),this.setValues(o)}}const t={gpuGeneration:!0,resolution:75,margin:.2,regenerate:()=>N(),mode:"raymarching",layer:0,surface:.1};let i,a,u,p,B,c,U,S,l,n,I,x,M,d,H;const L=new h;De();V();function De(){U=document.getElementById("output"),i=new j({antialias:!0}),i.setPixelRatio(window.devicePixelRatio),i.setSize(window.innerWidth,window.innerHeight),i.setClearColor(0,0),document.body.appendChild(i.domElement),u=new Y;const e=new $(16777215,1);e.position.set(1,1,1),u.add(e),u.add(new K(16777215,.2)),a=new Q(75,window.innerWidth/window.innerHeight,.1,50),a.position.set(1,1,2),a.far=100,a.updateProjectionMatrix(),c=new X(new J),u.add(c),new ce(a,i.domElement),B=new le,document.body.appendChild(B.dom),x=new _(new ge),M=new _(new be),d=new _(new we),H=new ue,new se().setMeshoptDecoder(xe).loadAsync("https://raw.githubusercontent.com/gkjohnson/3d-demo-data/main/models/stanford-bunny/bunny.glb").then(o=>{o.scene.updateMatrixWorld(!0);const r=new he(o.scene);return r.attributes=["position","normal"],r.useGroups=!1,l=r.generate().center(),H.generate(l,{maxLeafTris:1})}).then(o=>{S=o,I=new Z(l,new ee),u.add(I),N()}),z(),window.addEventListener("resize",function(){a.aspect=window.innerWidth/window.innerHeight,a.updateProjectionMatrix(),i.setSize(window.innerWidth,window.innerHeight)},!1)}function z(){p&&p.destroy(),t.layer=Math.min(t.resolution,t.layer),p=new de;const e=p.addFolder("generation");e.add(t,"gpuGeneration"),e.add(t,"resolution",10,200,1),e.add(t,"margin",0,1),e.add(t,"regenerate");const o=p.addFolder("display");o.add(t,"mode",["geometry","raymarching","layer","grid layers"]).onChange(()=>{z()}),t.mode==="layer"&&o.add(t,"layer",0,t.resolution,1),t.mode==="raymarching"&&o.add(t,"surface",-.2,.5)}function N(){const e=t.resolution,o=new h,r=new T,P=new ae,v=new T;l.boundingBox.getCenter(r),v.subVectors(l.boundingBox.max,l.boundingBox.min),v.x+=2*t.margin,v.y+=2*t.margin,v.z+=2*t.margin,o.compose(r,P,v),L.copy(o).invert(),c.box.copy(l.boundingBox),c.box.min.x-=t.margin,c.box.min.y-=t.margin,c.box.min.z-=t.margin,c.box.max.x+=t.margin,c.box.max.y+=t.margin,c.box.max.z+=t.margin,n&&n.dispose();const f=1/e,y=.5*f,k=window.performance.now();if(t.gpuGeneration){const m=i.extensions.get("OES_texture_float_linear");n=new te(e,e,e),n.texture.format=C,n.texture.type=m?q:oe,n.texture.minFilter=D,n.texture.magFilter=D,x.material.uniforms.bvh.value.updateFrom(S),x.material.uniforms.matrix.value.copy(o);for(let s=0;s<e;s++)x.material.uniforms.zValue.value=s*f+y,i.setRenderTarget(n,s),x.render(i);i.readRenderTargetPixels(n,0,0,1,1,new Float32Array(4)),i.setRenderTarget(null)}else{n=new ne(new Float32Array(e**3),e,e,e),n.format=C,n.type=q,n.minFilter=D,n.magFilter=D,n.needsUpdate=!0;const m=new T,s=new ie,E={};for(let g=0;g<e;g++)for(let b=0;b<e;b++)for(let w=0;w<e;w++){m.set(y+g*f-.5,y+b*f-.5,y+w*f-.5).applyMatrix4(o);const W=g+b*e+w*e*e,A=S.closestPointToPoint(m,E).distance;s.origin.copy(m),s.direction.set(0,0,1);const R=S.raycastFirst(s,re),O=R&&R.face.normal.dot(s.direction)>0;n.image.data[W]=O?-A:A}}const G=window.performance.now()-k;U.innerText=`${G.toFixed(2)}ms`,z()}function V(){if(B.update(),requestAnimationFrame(V),n){if(t.mode==="geometry")i.render(u,a);else if(t.mode==="layer"||t.mode==="grid layers"){let e;const o=M.material;n.isData3DTexture?(o.uniforms.layer.value=t.layer/n.image.width,o.uniforms.sdfTex.value=n,e=n):(o.uniforms.layer.value=t.layer/n.width,o.uniforms.sdfTex.value=n.texture,e=n.texture),o.uniforms.layers.value=e.image.width;const r=t.mode==="layer"?0:1;r!==o.defines.DISPLAY_GRID&&(o.defines.DISPLAY_GRID=r,o.needsUpdate=!0),M.render(i)}else if(t.mode==="raymarching"){a.updateMatrixWorld(),I.updateMatrixWorld();let e;n.isData3DTexture?e=n:e=n.texture;const{width:o,depth:r,height:P}=e.image;d.material.uniforms.sdfTex.value=e,d.material.uniforms.normalStep.value.set(1/o,1/P,1/r),d.material.uniforms.surface.value=t.surface,d.material.uniforms.projectionInverse.value.copy(a.projectionMatrixInverse),d.material.uniforms.sdfTransformInverse.value.copy(I.matrixWorld).invert().premultiply(L).multiply(a.matrixWorld),d.render(i)}}else return}
