import{B as q,b as z,W as K,c as U,d as O,D as _,A as j,P as N,G as J,M as P,e as Q,f as X,T as Y,g as Z,C as ee,h as re}from"./ExtendedTriangle-CtXgmvQ9.js";import{S as ne}from"./stats.min-GTpOrGrX.js";import{g as te}from"./lil-gui.module.min-Bc0DeA9g.js";import{i as A,e as oe,M as v,c as M,C as F,A as ae,S as ie}from"./MeshBVH-gvUqwvsz.js";import{G as se,W as de}from"./GenerateMeshBVHWorker-CvqnR6RV.js";import{M as T}from"./MeshBVHHelper-BGdsxYLH.js";import"./_commonjsHelpers-Cpj98o6Y.js";const G=typeof navigator<"u"?navigator.hardwareConcurrency:4;class le extends de{constructor(){const o=new Worker(new URL(""+new URL("parallelMeshBVH.worker-CbspdTLy.js",import.meta.url).href,import.meta.url),{type:"module"});if(super(o),this.name="ParallelMeshBVHWorker",this.maxWorkerCount=Math.max(G,4),!A())throw new Error("ParallelMeshBVHWorker: Shared Array Buffers are not supported.")}runTask(o,r,n={}){return new Promise((u,a)=>{if(!r.index&&!n.indirect&&oe(r,n),r.getAttribute("position").isInterleavedBufferAttribute||r.index&&r.index.isInterleavedBufferAttribute)throw new Error("ParallelMeshBVHWorker: InterleavedBufferAttribute are not supported for the geometry attributes.");o.onerror=p=>{a(new Error(`ParallelMeshBVHWorker: ${p.message}`))},o.onmessage=p=>{const{data:f}=p;if(f.error)a(new Error(f.error)),o.onmessage=null;else if(f.serialized){const{serialized:x,position:$}=f,k=v.deserialize(x,r,{setIndex:!1}),D={setBoundingBox:!0,...n};if(r.attributes.position.array=$,x.index)if(r.index)r.index.array=x.index;else{const L=new q(x.index,1,!1);r.setIndex(L)}D.setBoundingBox&&(r.boundingBox=k.getBoundingBox(new z)),n.onProgress&&n.onProgress(f.progress),u(k),o.onmessage=null}else n.onProgress&&n.onProgress(f.progress)};const d=r.index?r.index.array:null,h=r.attributes.position.array;o.postMessage({operation:"BUILD_BVH",maxWorkerCount:this.maxWorkerCount,index:M(d,SharedArrayBuffer),position:M(h,SharedArrayBuffer),options:{...n,onProgress:null,includedProgressCallback:!!n.onProgress,groups:[...r.groups]}})})}}class ue{constructor(){if(A())return new le;{console.warn("ParallelMeshBVHWorker: SharedArrayBuffers not supported. Falling back to single-threaded GenerateMeshBVHWorker.");const o=new se;return o.maxWorkerCount=G,o}}}const C=typeof SharedArrayBuffer<"u",e={useWebWorker:!0,maxWorkerCount:C?navigator.hardwareConcurrency:1,strategy:F,radius:1,tube:.3,tubularSegments:500,radialSegments:500,p:3,q:5,displayHelper:!1,helperDepth:10};let m,c,w,i,V,g,t,s,y,W,S,E,R,B,b=!1;pe();I();function pe(){W=document.getElementById("output"),S=document.getElementById("loading-container"),E=document.querySelector("#loading-container .bar"),R=document.querySelector("#loading-container .text"),m=new K({antialias:!0}),m.setPixelRatio(window.devicePixelRatio),m.setSize(window.innerWidth,window.innerHeight),m.setClearColor(16763432,1),document.body.appendChild(m.domElement),w=new U,w.fog=new O(16763432,20,60);const o=new _(16777215,3);o.position.set(1,1,1),w.add(o),w.add(new j(11583173,2.5)),c=new N(75,window.innerWidth/window.innerHeight,.1,50),c.position.set(0,0,4),c.far=100,c.updateProjectionMatrix(),V=new re,y=new ne,document.body.appendChild(y.dom),s=new J,w.add(s);for(let a=0;a<400;a++){const d=new P(new Q(1,32,32),new X);d.position.set(Math.random()-.5,Math.random()-.5,Math.random()-.5).multiplyScalar(70),d.scale.setScalar(Math.random()*.3+.1),s.add(d)}B=new ue,window.WORKER=B,g=new te;const r=g.addFolder("helper");r.add(e,"displayHelper").name("enabled").onChange(a=>{a&&t&&t.update()}),r.add(e,"helperDepth",1,50,1).onChange(a=>{t&&(t.depth=a,t.update())}),r.open();const n=g.addFolder("knot");n.add(e,"radius",.5,2,.01),n.add(e,"tube",.2,1.2,.01),n.add(e,"tubularSegments",50,2e3,1),n.add(e,"radialSegments",5,2e3,1),n.add(e,"p",1,10,1),n.add(e,"q",1,10,1),n.open();const u=g.addFolder("bvh");u.add(e,"useWebWorker"),u.add(e,"maxWorkerCount",1,16,1).disable(!C),u.add(e,"strategy",{CENTER:F,AVERAGE:ae,SAH:ie}),g.add({regenerateKnot:H},"regenerateKnot").name("regenerate"),H(),window.addEventListener("resize",function(){c.aspect=window.innerWidth/window.innerHeight,c.updateProjectionMatrix(),m.setSize(window.innerWidth,window.innerHeight)},!1)}function H(){if(b)return;b=!0,i&&(i.material.dispose(),i.geometry.dispose(),s.remove(i),s.remove(t));const l=window.performance.now(),o=window.performance.now();i=new P(new Y(e.radius,e.tube,e.tubularSegments,e.radialSegments,e.p,e.q),new Z({color:new ee(5093036),roughness:.75}));const r=window.performance.now()-o,n=window.performance.now(),u={strategy:e.strategy};let a;if(e.useWebWorker){const d=h=>{const p=(h*100).toFixed(0);S.style.visibility="visible",E.style.width=`${p}%`,R.innerText=`${p}%`};B.maxWorkerCount=e.maxWorkerCount,B.generate(i.geometry,{onProgress:d,...u}).then(h=>{S.style.visibility="hidden",i.geometry.boundsTree=h,s.add(i);const p=window.performance.now()-n;b=!1,t=new T(i,0),t.depth=e.helperDepth,e.displayHelper&&t.update(),s.add(t),W.textContent=`Geometry Generation Time  : ${r.toFixed(3)}ms
BVH Generation Time       : ${p.toFixed(3)}ms
Frame Stall Time          : ${a.toFixed(3)}
SharedArrayBuffer Support : ${C}`}),a=window.performance.now()-l}else{i.geometry.boundsTree=new v(i.geometry,u),a=window.performance.now()-l,s.add(i);const d=window.performance.now()-n;b=!1,t=new T(i),t.depth=e.helperDepth,t.update(),s.add(t),W.textContent=`Geometry Generation Time  : ${r.toFixed(3)}ms
BVH Generation Time       : ${d.toFixed(3)}ms
Frame Stall Time          : ${a.toFixed(3)}`}}function I(){y.update(),requestAnimationFrame(I);let l=V.getDelta();s.rotation.x+=.4*l,s.rotation.y+=.6*l,t&&(t.visible=e.displayHelper),m.render(w,c)}
