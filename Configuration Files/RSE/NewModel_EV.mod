set P11:={2}; 
set P12:={4}; 
set P21:={6}; 
set P22:={7,8};  
set P:={ 2, 4, 6, 7, 8};
set K  := {1,2};
set J  := {1..10};
set P2  :={1..8};  
set I  :={1..2};  
set T  := {1..10}; 
set C  := {1..4}; 
  
param y    {p in P}, >=0,default 0;  
param red  {p in P}, >=0,default 0;  
param gmin {p in P}, >=0,default 0;  
param gmax {p in P}, >=0,default 0;  
param init1,default 0;  
param init2,default 0;  
param Grn1, default 0;  
param Grn2, default 0;  
param SP1,  integer,default 0;  
param SP2,  integer,default 0;  
param M:=9999,integer;  
param alpha:=100,integer;  
param Rl{p in P, j in J}, >=0,  default 0;  
param Ru{p in P, j in J}, >=0,  default 0;  
param Tq{p in P, j in J}, >=0,  default 0;  
param Cl{p in P, c in C}, >=0,  default 0;  
param Cu{p in P, c in C}, >=0,  default 0;  
  
param current,>=0,default 0;   # current is  mod(time,cycle)  
param coord, :=0; 	# this paramter indicated where we consider coorrdination or not  
param isItCoordinated, :=(if coord>0 then 1 else 0); 
param SumOfMinGreen1, :=  ( sum{p in P11} gmin[p] ) + ( sum{p in P12} gmin[p] ) ; 
param SumOfMinGreen2, :=  ( sum{p in P21} gmin[p] ) + ( sum{p in P22} gmin[p] ) ; 
param cycle, :=100;    # if we have coordination, the cycle length  
param coordphase1,:=2;    # if we have coordination, thecoordinated phase in ring1  
param coordphase2,:=6;    # if we have coordination, thecoordinated phase in ring2  
param isCurPhCoord1, :=(if coordphase1=SP1 then 1 else 0);  
param isCurPhCoord2, :=(if coordphase2=SP2 then 1 else 0);  
param earlyReturn,   := (if ( ( isItCoordinated=1 and ( current> 0.75*cycle ) and coordphase1>0)  or 	(isItCoordinated=1 and ( current> 0.75*cycle ) and coordphase2>0) ) then 1 else 0); 
param afterSplit,    := (if ( ( isItCoordinated=1 and  Grn1>=gmax[coordphase1] and SP1=coordphase1 and current>= gmax[coordphase1]/1.1 and current<= gmax[coordphase1])  or 	(isItCoordinated=1 and ( Grn2>=gmax[coordphase2] ) and SP2=coordphase2 and current>= gmax[coordphase1]/1.1 and current<=gmax[coordphase2]) ) then 1 else 0); 
param inSplit,       := (if ( ( isItCoordinated=1 and  SP1=coordphase1 and current<gmax[coordphase1]/1.1  )  or (isItCoordinated=1 and SP2=coordphase2 and current< gmax[coordphase1]/1.1 ) ) then 1 else 0); 
param PrioType { t in T}, >=0, default 0;  
param PrioWeigth { t in T}, >=0, default 0;  
param priorityType{j in J}, >=0, default 0;  
param priorityTypeWeigth{j in J, t in T}, := (if (priorityType[j]=t) then PrioWeigth[t] else 0);  
param Arr{p in P, i in I}, >=0,  default 0;  
param active_arr{p in P, i in I}, integer, :=(if Arr[p,i]>0 then 1 else 0); 
param SumOfActiveArr, := (if (sum{p in P, i in I} Arr[p,i])>0 then (sum{p in P, i in I} Arr[p,i]) else 1); 
  
param active_pj{p in P, j in J}, integer, :=(  if Rl[p,j]>0 then	1  else	0);  
param active_coord{p in P, c in C}, integer, :=(  if Cl[p,c]>0 then	1  else	0);  
param coef{p in P,k in K}, integer,:=(   if ((((p<SP1 and p<5) or (p<SP2 and p>4 )) and k==1))then   	0   else   	1);  
param MinGrn1{p in P,k in K},:=(  if (((p==SP1 and p<5) and k==1))then   	Grn1   else   	0);  
param MinGrn2{p in P,k in K},:=(  if (((p==SP2 and p>4 ) and k==1))then   	Grn2   else   	0);  
param ReqNo:=sum{p in P,j in J} active_pj[p,j] + isItCoordinated*sum{p in P,c in C} active_coord[p,c];  
param queue_pj{p in P, j in J}, integer, :=(  if Tq[p,j]>0 then	1  else	0);  
  
var t{p in P,k in K}, >=0;    # starting time vector  
var g{p in P,k in K}, >=0;  
var v{p in P,k in K}, >=0;  
var d{p in P,j in J}, >=0;  
var theta{p in P,j in J}, binary;  
var PriorityDelay,>=0;  
 
var miu{p in P,i in I}, binary; 
var rd{p in P,i in I}, >=0; 
var tmiu{p in P, i in I}, >=0; 
var TotRegVehDel, >=0; 
var QueClrTime{p in P, i in I},>=0; 
var ttheta{p in P,j in J}, >=0; 
var zeta1, binary;
var zetatl , >=0;
var zeta2, binary;
var zetatu , >=0;
var gamma1, binary;
var gammatl , >=0;
var gamma2, binary;
var gammatu , >=0;
var coord1delay , >=0;
var coord2delay , >=0;
  
#================ Begin of cycle 1======================#  
s.t. initial{p in P:(p<SP1) or (p<SP2 and p>4)}: t[p,1]=0;  
s.t. initial1{p in P:p=SP1}: t[p,1]=init1;  
s.t. initial2{p in P:p=SP2}: t[p,1]=init2;  

 # constraints in the same cycle in same P??  
s.t. Prec_11_11_c1{p in P11: (p+1)in P11 and p>=SP1  }:  t[p+1,1]=t[p,1]+v[p,1];  
s.t. Prec_12_12_c1{p in P12: (p+1)in P12 and p>=SP1  }:  t[p+1,1]=t[p,1]+v[p,1];  
s.t. Prec_21_21_c1{p in P21: (p+1)in P21 and p>=SP2  }:  t[p+1,1]=t[p,1]+v[p,1];  
s.t. Prec_22_22_c1{p in P22: (p+1)in P22 and p>=SP2  }:  t[p+1,1]=t[p,1]+v[p,1];  
  
# constraints in the same cycle in connecting   
s.t. Prec_11_12_c1{p in P12: (card(P12)+p)<=5 and p>SP1  }:  t[p,1]=t[2,1]+v[2,1];  
s.t. Prec_11_22_c1{p in P22: (card(P22)+p)<=9 and p>SP2  }:  t[p,1]=t[2,1]+v[2,1];  
s.t. Prec_21_12_c1{p in P12: (card(P12)+p)<=5 and p>SP1  }:  t[p,1]=t[6,1]+v[6,1];  
s.t. Prec_21_22_c1{p in P22: (card(P22)+p)<=9 and p>SP2  }:  t[p,1]=t[6,1]+v[6,1];  
  
#================ END of cycle 1======================#  
  
# constraints in the same cycle in same P??  
s.t. Prec_11_11_c23{p in P11, k in K: (p+1)in P11 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  
s.t. Prec_12_12_c23{p in P12, k in K: (p+1)in P12 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  
s.t. Prec_21_21_c23{p in P21, k in K: (p+1)in P21 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  
s.t. Prec_22_22_c23{p in P22, k in K: (p+1)in P22 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  
  
# constraints in the same cycle in connecting   
s.t. Prec_11_12_c23{p in P12, k in K: (card(P12)+p)=5 and k>1 }:  t[p,k]=t[2,k]+v[2,k];
s.t. Prec_11_22_c23{p in P22, k in K: (card(P22)+p)=9 and k>1 }:  t[p,k]=t[2,k]+v[2,k];  
s.t. Prec_21_12_c23{p in P12, k in K: (card(P12)+p)=5 and k>1 }:  t[p,k]=t[6,k]+v[6,k];  
s.t. Prec_21_22_c23{p in P22, k in K: (card(P22)+p)=9 and k>1 }:  t[p,k]=t[6,k]+v[6,k];  
  
# constraints in connecting in different cycles  
s.t. Prec_12_11_c23{p in P11, k in K: (card(P11)+p+1)=4 and k>1 }:    t[p,k]=t[4,k-1]+v[4,k-1];  
s.t. Prec_22_11_c23{p in P11, k in K: (card(P11)+p+1+4)=8 and k>1 }:  t[p,k]=t[8,k-1]+v[8,k-1];  
s.t. Prec_12_21_c23{p in P21, k in K: (card(P21)+p+1-4)=4 and k>1 }:  t[p,k]=t[4,k-1]+v[4,k-1];  
s.t. Prec_22_21_c23{p in P21, k in K: (card(P21)+p+1)=8 and k>1 }:    t[p,k]=t[8,k-1]+v[8,k-1];  
  
#==================================================#  
  
s.t. RD: PriorityDelay=( sum{p in P,j in J, tt in T} (priorityTypeWeigth[j,tt]*active_pj[p,j]*d[p,j] ) )  + PrioWeigth[6]*(coord1delay + coord2delay)/2 ;  
  
s.t. c0: coord1delay=isItCoordinated*((coef[coordphase1,1]-afterSplit)*(( zetatl - Cl[coordphase1,1]*zeta1 ) + (-zetatu + Cu[coordphase1,1]*zeta2)) +(1-coef[coordphase1,1] +afterSplit)*(( zetatl - Cl[coordphase1,1]*zeta1 ) + (-zetatu + Cu[coordphase1,1]*zeta2)) ); 
s.t. cc0: coord2delay=isItCoordinated*((coef[coordphase2,1]-afterSplit)*(( gammatl - Cl[coordphase2,2]*gamma1 ) + (-gammatu + Cu[coordphase2,2]*gamma2)) +(1-coef[coordphase2,1]+afterSplit)*(( gammatl - Cl[coordphase2,2]*gamma1 ) + (-gammatu + Cu[coordphase2,2]*gamma2)) ); 
s.t. c1: M*zeta1>=isItCoordinated*(coef[coordphase1,1]-afterSplit)*( t[coordphase1,1]-Cl[coordphase1,1] ); 
s.t. c2: isItCoordinated*(coef[coordphase1,1]-afterSplit)* ( Cl[coordphase1,1]-t[coordphase1,1]) <= M*(1-zeta1);
s.t. c3: isItCoordinated*(coef[coordphase1,1]-afterSplit)* ( t[coordphase1,1]-M*(1-zeta1) ) <= zetatl; 
s.t. c4: isItCoordinated*(coef[coordphase1,1]-afterSplit)* zetatl <=( t[coordphase1,1]+M*(1-zeta1) ) ; 
s.t. c5: isItCoordinated*(coef[coordphase1,1]-afterSplit)* zetatl <=M*zeta1; 
s.t. c6: M*zeta2>=isItCoordinated*(coef[coordphase1,1]-afterSplit)*( Cu[coordphase1,1]-(g[coordphase1,1]+t[coordphase1,1]) ); 
s.t. c7: isItCoordinated*(coef[coordphase1,1]-afterSplit)*( (g[coordphase1,1]+t[coordphase1,1])-Cu[coordphase1,1] )<= M*(1-zeta2); 
s.t. c8: isItCoordinated*(coef[coordphase1,1]-afterSplit)*( (g[coordphase1,1]+t[coordphase1,1])-M*(1-zeta2) ) <= zetatu; 
s.t. c9: isItCoordinated*(coef[coordphase1,1]-afterSplit)*zetatu <= ( (g[coordphase1,1]+t[coordphase1,1]) + M*(1-zeta2) ) ; 
s.t. c10: isItCoordinated*(coef[coordphase1,1]-afterSplit)*zetatu<=M*zeta2; 
s.t. c11: M*zeta1>=isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( t[coordphase1,2]-Cl[coordphase1,1] ); 
s.t. c12: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( Cl[coordphase1,1]-t[coordphase1,2]) <= M*(1-zeta1); 
s.t. c13: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( t[coordphase1,2]-M*(1-zeta1) )<= zetatl; 
s.t. c14: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) *zetatl<= ( t[coordphase1,2]+M*(1-zeta1) ) ; 
s.t. c15: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) *zetatl<=M*zeta1; 
s.t. c16: M*zeta2>=isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( Cu[coordphase1,1]-(g[coordphase1,2]+t[coordphase1,2]) ); 
s.t. c17: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( (g[coordphase1,2]+t[coordphase1,2])-Cu[coordphase1,1] )<= M*(1-zeta2); 
s.t. c18: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) * ( (g[coordphase1,2]+t[coordphase1,2])-M*(1-zeta2) ) <= zetatu; 
s.t. c19: isItCoordinated*(1-coef[coordphase1,1+afterSplit]) *zetatu <=  ( (g[coordphase1,2]+t[coordphase1,2]) + M*(1-zeta2) ) ; 
s.t. c20: isItCoordinated*(1-coef[coordphase1,1]+afterSplit) *zetatu<=M*zeta2; 
s.t. cc1: M*gamma1>=isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( t[coordphase2,1]-Cl[coordphase2,2] ); 
s.t. cc2: isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( Cl[coordphase2,2]-t[coordphase2,1] ) <= M*(1-gamma1); 
s.t. cc3: isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( t[coordphase2,1]-M*(1-gamma1) )<= gammatl; 
s.t. cc4: isItCoordinated*(coef[coordphase2,1]-afterSplit)* gammatl<=( t[coordphase2,1]+M*(1-gamma1) ) ; 
s.t. cc5: isItCoordinated*(coef[coordphase2,1]-afterSplit)* gammatl<=M*gamma1; 
s.t. cc6: M*gamma2>=isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( Cu[coordphase2,2]-(g[coordphase2,1]+t[coordphase2,1]) ); 
s.t. cc7: isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( (g[coordphase2,1]+t[coordphase2,1])-Cu[coordphase2,2])<= M*(1-gamma2); 
s.t. cc8: isItCoordinated*(coef[coordphase2,1]-afterSplit)* ( (g[coordphase2,1]+t[coordphase2,1])-M*(1-gamma2) ) <= gammatu; 
s.t. cc9: isItCoordinated*(coef[coordphase2,1]-afterSplit)* gammatu <= ( (g[coordphase2,1]+t[coordphase2,1]) + M*(1-gamma2) ) ; 
s.t. cc10: isItCoordinated*(coef[coordphase2,1]-afterSplit)* gammatu<=M*gamma2; 
s.t. cc11: M*gamma1>=isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( t[coordphase2,2]-Cl[coordphase2,2] ); 
s.t. cc12: isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( Cl[coordphase2,2]-t[coordphase2,2] ) <= M*(1-gamma1); 
s.t. cc13: isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( t[coordphase2,2]-M*(1-gamma1) )<= gammatl; 
s.t. cc14: isItCoordinated*(1-coef[coordphase2,1]+afterSplit)*gammatl<= ( t[coordphase2,2]+M*(1-gamma1) ) ; 
s.t. cc15: isItCoordinated*(1-coef[coordphase2,1]+afterSplit)*gammatl <= M*gamma1; 
s.t. cc16: M*gamma2>=isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( Cu[coordphase2,2]-(g[coordphase2,2]+t[coordphase2,2]) ); 
s.t. cc17: isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( (g[coordphase2,2]+t[coordphase2,2])-Cu[coordphase2,2])<= M*(1-gamma2); 
s.t. cc18: isItCoordinated*(1-coef[coordphase2,1]+afterSplit) * ( (g[coordphase2,2]+t[coordphase2,2])-M*(1-gamma2) ) <= gammatu; 
s.t. cc19: isItCoordinated*(1-coef[coordphase2,1]+afterSplit)*gammatu <=  ( (g[coordphase2,2]+t[coordphase2,2]) + M*(1-gamma2) ) ; 
s.t. cc20: isItCoordinated*(1-coef[coordphase2,1]+afterSplit)*gammatu <= M*gamma2; 
s.t. TotRegVehDelay: TotRegVehDel=(sum{p in P, i in I} active_arr[p,i]*Arr[p,i]*rd[p,i])/SumOfActiveArr;  
  
s.t. PhaseLen{p in P, k in K}:  v[p,k]=(g[p,k]+y[p]+red[p])*coef[p,k];  
  
s.t. PhaseLen2{p in P, k in K}:  v[p,k]*coef[p,k]>=g[p,k]; 
  
s.t. GrnMax{p in P ,k in K:  ( (p!= coordphase1 and p!=coordphase2 and isItCoordinated=1)		 or isItCoordinated=0 ) }:  g[p,k]<=(gmax[p]-MinGrn1[p,k]-MinGrn2[p,k])*coef[p,k];    
s.t. GrnMax2:  (1-earlyReturn)*g[2,1]<=(1-earlyReturn)*(gmax[2]-((inSplit+afterSplit)*(current)+(1-afterSplit-inSplit)*(MinGrn1[2,1]+MinGrn2[2,1])));   
s.t. GrnMax3:  (1-earlyReturn)*g[6,1]<=(1-earlyReturn)*(gmax[6]-((inSplit+afterSplit)*(current)+(1-afterSplit-inSplit)*(MinGrn1[6,1]+MinGrn2[6,1])));   
s.t. GrnMax4{p in P: (p=coordphase1 or p=coordphase2)}:  g[p,2]<=gmax[p];   
s.t. GrnMin{p in P,k in K}:  g[p,k]>=(gmin[p]-MinGrn1[p,k]-MinGrn2[p,k])*coef[p,k];   
  
	s.t. PrioDelay1{p in P,j in J: active_pj[p,j]>0 }:    d[p,j]>=t[p,1]-Rl[p,j];  
  
 s.t. PrioDelay2{p in P,j in J: active_pj[p,j]>0 }:    M*theta[p,j]>=Ru[p,j]-(t[p,1]+g[p,1]); 
  
 s.t. PrioDelay3{p in P,j in J: active_pj[p,j]>0 }:    d[p,j]>= ttheta[p,j]-Rl[p,j]*theta[p,j]; 
  
 s.t. PrioDelay4{p in P,j in J: active_pj[p,j]>0 }:    g[p,1]>= (Ru[p,j]-Rl[p,j])*(1-theta[p,j]); 
  
 s.t. PrioDelay5{p in P, j in J: active_pj[p,j]>0}:    ttheta[p,j]<=M*theta[p,j]; 
  
 s.t. PrioDelay6{p in P, j in J: active_pj[p,j]>0}:    t[p,2]-M*(1-theta[p,j])<=ttheta[p,j]; 
  
 s.t. PrioDelay7{p in P, j in J: active_pj[p,j]>0}:    t[p,2]+M*(1-theta[p,j])>=ttheta[p,j]; 
  
 s.t. PrioDelay8{p in P, j in J: active_pj[p,j]>0}:   g[p,2]>=(Ru[p,j]-Rl[p,j])*theta[p,j]; 
   
s.t. PrioDelay9 {p in P, j in J: active_pj[p,j]>0}:   Ru[p,j]*theta[p,j] <= ( t[p,2]+g[p,2]) ; 
   
minimize delay: PriorityDelay  ;
  
solve;  
  
printf " " > "/nojournal/bin/Results.txt";  
printf "%3d  %3d \n ",SP1, SP2 >>"/nojournal/bin/Results.txt";  
printf "%5.2f  %5.2f %5.2f  %5.2f \n ",init1, init2,Grn1,Grn2 >>"/nojournal/bin/Results.txt";  
for {k in K}   
 { 
     for {p in P2} 
        { 
           printf "%5.2f  ", if(p in P)  then v[p,k] else 0  >>"/nojournal/bin/Results.txt";   
        } 
        printf " \n ">>"/nojournal/bin/Results.txt";
 } 
  
for {k in K}   
 { 
     for {p in P2} 
        { 
           printf "%5.2f  ", if(p in P)  then g[p,k] else 0  >>"/nojournal/bin/Results.txt";   
        } 
        printf " \n ">>"/nojournal/bin/Results.txt";
 } 
  
printf "%3d \n ", ReqNo >>"/nojournal/bin/Results.txt";  
  
for {p in P,j in J : Rl[p,j]>0}  
 {  
   printf "%d  %5.2f  %5.2f  %5.2f %d \n ", (p+ 10*(theta[p,j])), Rl[p,j],Ru[p,j], d[p,j] , priorityType[j] >>"/nojournal/bin/Results.txt";
 } 
printf "%5.2f \n ", PriorityDelay >>"/nojournal/bin/Results.txt"; 
printf "%5.2f \n ", TotRegVehDel >>"/nojournal/bin/Results.txt";  
 printf " \n ">>"/nojournal/bin/Results.txt";
end;
