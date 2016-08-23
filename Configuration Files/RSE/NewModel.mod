set P11:={1,2};  
set P12:={3,4};
set P21:={5,6};
set P22:={7,8};
set P:={ 1, 2, 3, 4, 5, 6, 7, 8};
set K  := {1,2};
set J  := {1..10};
set I  :={1..130};
set L  :={1..20};
set P2  :={1..8};
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
param Cl{p in P, c in C}, >=0,  default 0;
param Cu{p in P, c in C}, >=0,  default 0;
  
param current,>=0,default 0;
param coord, :=0;
param isItCoordinated, :=(if coord>0 then 1 else 0);
param SumOfMinGreen1, :=  ( sum{p in P11} gmin[p] ) + ( sum{p in P12} gmin[p] ) ;
param SumOfMinGreen2, :=  ( sum{p in P21} gmin[p] ) + ( sum{p in P22} gmin[p] ) ;
param cycle, :=100;
param coordphase1,:=2;
param coordphase2,:=6;
param isCurPhCoord1, :=(if coordphase1=SP1 then 1 else 0);
param isCurPhCoord2, :=(if coordphase2=SP2 then 1 else 0);
param earlyReturn,   := (if ( ( isItCoordinated=1 and ( current> 0.75*cycle ) and coordphase1>0)  or 	(isItCoordinated=1 and ( current> 0.75*cycle ) and coordphase2>0) ) then 1 else 0); 
param afterSplit,    := (if ( ( isItCoordinated=1 and  Grn1>=gmax[coordphase1] and SP1=coordphase1 and current>= gmax[coordphase1]/1.25 and current<= gmax[coordphase1])  or 	(isItCoordinated=1 and ( Grn2>=gmax[coordphase2] ) and SP2=coordphase2 and current>= gmax[coordphase1]/1.25 and current<=gmax[coordphase2]) ) then 1 else 0); 
param inSplit,       := (if ( ( isItCoordinated=1 and  SP1=coordphase1 and current<gmax[coordphase1]/1.25  )  or (isItCoordinated=1 and SP2=coordphase2 and current< gmax[coordphase1]/1.25 ) ) then 1 else 0); 
param PrioType { t in T}, >=0, default 0;  
param PrioWeigth { t in T}, >=0, default 0;  
param priorityType{j in J}, >=0, default 0;  
param priorityTypeWeigth{j in J, t in T}, := (if (priorityType[j]=t) then PrioWeigth[t] else 0);  
param s ,>=0,default 0;
param qSp ,>=0,default 0;
param Ar{i in I}, >=0,  default 0;
param Tq{i in I}, >=0,  default 0;
param active_arr{i in I}, integer, :=(if Ar[i]>0 then 1 else 0);
param SumOfActiveArr, := (if (sum{i in I} Ar[i])>0 then (sum{i in I} Ar[i]) else 1);
param Ve{i in I}, >=0,  default 0;
param Ln{i in I}, >=0,  default 0;
param Ph{i in I}, >=0,  default 0;
param L0{l in L}, >=0,  default 0;
param LaPhSt{l in L}, integer;
param Ratio{i in I}, >=0,  default 0;
param indic{i in I},:= (if (Ratio[i]-L0[Ln[i]]/(s-qSp))> 0 then 1 else 0);
  
param active_pj{p in P, j in J}, integer, :=(if Rl[p,j]>0 then 1 else	0);
param active_coord{p in P, c in C}, integer, :=(if Cl[p,c]>0 then	1 else	0);
param coef{p in P,k in K}, integer,:=(if ((((p<SP1 and p<5) or (p<SP2 and p>4 )) and k==1))then 0 else 1);
param MinGrn1{p in P,k in K},:=(if (((p==SP1 and p<5) and k==1))then Grn1 else 0);
param MinGrn2{p in P,k in K},:=(if (((p==SP2 and p>4 ) and k==1))then	Grn2 else 0);
param ReqNo:=sum{p in P,j in J} active_pj[p,j] + isItCoordinated*sum{p in P,c in C} active_coord[p,c];  
  
var t{p in P,k in K}, >=0;    # starting time vector  
var g{p in P,k in K}, >=0;  
var v{p in P,k in K}, >=0;  
var d{p in P,j in J}, >=0;  
var theta{p in P,j in J}, binary;  
var ttheta{p in P,j in J}, >=0; 
var PriorityDelay,>=0;  
 
var miu{i in I}, binary; 
var rd{i in I}, >=0; 
var q{i in I}, >=0; 
var qq{i in I}; 
var qIndic{i in I}, binary; 
var qmiu{i in I}, >=0; 
var tmiu{ i in I}, >=0; 
var gmiu{ i in I}, >=0; 
var del{i in I}, binary; 
var TotRegVehDel, >=0; 
var delT1{i in I}, >=0; 
var delT2{i in I}, >=0; 
var ze{i in I}, binary; 
var delZe{i in I}, binary; 
var delZeT1{i in I}, >=0; 
var delZeT2{i in I}, >=0; 
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
  
s.t. initial{p in P:(p<SP1) or (p<SP2 and p>4)}: t[p,1]=0;  
s.t. initial1{p in P:p=SP1}: t[p,1]=init1;  
s.t. initial2{p in P:p=SP2}: t[p,1]=init2;  
s.t. Prec_11_11_c1{p in P11: (p+1)in P11 and p>=SP1  }:  t[p+1,1]=t[p,1]+v[p,1];  
s.t. Prec_12_12_c1{p in P12: (p+1)in P12 and p>=SP1  }:  t[p+1,1]=t[p,1]+v[p,1];  
s.t. Prec_21_21_c1{p in P21: (p+1)in P21 and p>=SP2  }:  t[p+1,1]=t[p,1]+v[p,1];  
s.t. Prec_22_22_c1{p in P22: (p+1)in P22 and p>=SP2  }:  t[p+1,1]=t[p,1]+v[p,1];  
  
s.t. Prec_11_12_c1{p in P12: (card(P12)+p)<=5 and p>SP1  }:  t[p,1]=t[2,1]+v[2,1];  
s.t. Prec_11_22_c1{p in P22: (card(P22)+p)<=9 and p>SP2  }:  t[p,1]=t[2,1]+v[2,1];  
s.t. Prec_21_12_c1{p in P12: (card(P12)+p)<=5 and p>SP1  }:  t[p,1]=t[6,1]+v[6,1];  
s.t. Prec_21_22_c1{p in P22: (card(P22)+p)<=9 and p>SP2  }:  t[p,1]=t[6,1]+v[6,1];  
  
  
s.t. Prec_11_11_c23{p in P11, k in K: (p+1)in P11 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  
s.t. Prec_12_12_c23{p in P12, k in K: (p+1)in P12 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  
s.t. Prec_21_21_c23{p in P21, k in K: (p+1)in P21 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  
s.t. Prec_22_22_c23{p in P22, k in K: (p+1)in P22 and k>1  }:  t[p+1,k]=t[p,k]+v[p,k];  
  
s.t. Prec_11_12_c23{p in P12, k in K: (card(P12)+p)=5 and k>1 }:  t[p,k]=t[2,k]+v[2,k];
s.t. Prec_11_22_c23{p in P22, k in K: (card(P22)+p)=9 and k>1 }:  t[p,k]=t[2,k]+v[2,k];  
s.t. Prec_21_12_c23{p in P12, k in K: (card(P12)+p)=5 and k>1 }:  t[p,k]=t[6,k]+v[6,k];  
s.t. Prec_21_22_c23{p in P22, k in K: (card(P22)+p)=9 and k>1 }:  t[p,k]=t[6,k]+v[6,k];  
  
s.t. Prec_12_11_c23{p in P11, k in K: (card(P11)+p+1)=4 and k>1 }:    t[p,k]=t[4,k-1]+v[4,k-1];  
s.t. Prec_22_11_c23{p in P11, k in K: (card(P11)+p+1+4)=8 and k>1 }:  t[p,k]=t[8,k-1]+v[8,k-1];  
s.t. Prec_12_21_c23{p in P21, k in K: (card(P21)+p+1-4)=4 and k>1 }:  t[p,k]=t[4,k-1]+v[4,k-1];  
s.t. Prec_22_21_c23{p in P21, k in K: (card(P21)+p+1)=8 and k>1 }:    t[p,k]=t[8,k-1]+v[8,k-1];  
  
  
s.t. RD: PriorityDelay=( sum{p in P,j in J, tt in T} (priorityTypeWeigth[j,tt]*active_pj[p,j]*d[p,j] ) )  + PrioWeigth[6]*(coord1delay + coord2delay)/2 ;  
  
s.t. TotRegVehDelay: TotRegVehDel=(sum{i in I} active_arr[i]*rd[i])/SumOfActiveArr;  
  
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
   
s.t. RVehD1{i in I: (active_arr[i]>0)}: rd[i] >= t[Ph[i],1]+q[i]/s-(Ar[i]-q[i])/Ve[i]+Tq[i];  
 s.t. RVehD2{i in I: (active_arr[i]>0)} : M*miu[i] >= q[i]/s + Ar[i]/Ve[i] -g[Ph[i],1];  
 s.t. RVehD3{i in I: (active_arr[i]>0 )} : rd[i] >= tmiu[i] + qmiu[i]/s - gmiu[i] - Ar[i]*miu[i]/Ve[i] + qmiu[i]/Ve[i] + Tq[i]; 
 s.t. RVehD5{i in I,ii in I: (active_arr[i]>0 and active_arr[ii]>0  and Ln[i]=Ln[ii] and Ar[i]<Ar[ii])} : miu[i] <= miu[ii] ; 
 s.t. RVehD6{i in I: (active_arr[i]>0)}: tmiu[i]<=M*miu[i];   
 s.t. RVehD7{p in P,i in I: (active_arr[i]>0 and p=Ph[i])}: t[p,2]-M*(1-miu[i])<=tmiu[i];   
 s.t. RVehD8{p in P,i in I: (active_arr[i]>0 and p=Ph[i])}: t[p,2]+M*(1-miu[i])>=tmiu[i]; 
 s.t. RVehD9{i in I: active_arr[i]>0 }: gmiu[i]<=M*miu[i];   
 s.t. RVehD10{p in P,i in I: (active_arr[i]>0 and p=Ph[i])}: g[p,1]-M*(1-miu[i])<=gmiu[i];   
 s.t. RVehD11{p in P,i in I: (active_arr[i]>0 and p=Ph[i])}: g[p,1]+M*(1-miu[i])>=gmiu[i]; 
 s.t. RVehD12{i in I: (active_arr[i]>0)}: qmiu[i]<=M*miu[i];   
 s.t. RVehD13{i in I: (active_arr[i]>0)}: q[i]-M*(1-miu[i])<=qmiu[i];   
 s.t. RVehD14{i in I: (active_arr[i]>0)}: q[i]+M*(1-miu[i])>=qmiu[i]; 
 s.t. RVehD15{i in I,k in K: (active_arr[i]>0)}: qq[i]=((LaPhSt[Ln[i]])*(L0[Ln[i]]+Ratio[i]*qSp-g[Ph[i],1]*s)*indic[i])+((1-LaPhSt[Ln[i]])*(L0[Ln[i]]*del[i]+Ratio[i]*qSp*del[i]-s*Ratio[i]*del[i]+s*((1-coef[Ph[i],1])*delT2[i]+coef[Ph[i],1]*delT1[i])-s*Ratio[i]*delZe[i]-s*((1-coef[Ph[i],1])*delZeT2[i]+coef[Ph[i],1]*delZeT1[i]))); 
 s.t. RVehD16{i in I: (active_arr[i]>0)}: coef[Ph[i],1]*(t[Ph[i],1]-M*(1-del[i]))<=coef[Ph[i],1]*delT1[i];  
 s.t. RVehD17{i in I: (active_arr[i]>0)}: coef[Ph[i],1]*(t[Ph[i],1]+M*(1-del[i]))>=coef[Ph[i],1]*delT1[i]; 
 s.t. RVehD18{i in I: (active_arr[i]>0)}: coef[Ph[i],1]*delT1[i]<=M*del[i]; 
 s.t. RVehD19{i in I: (active_arr[i]>0)}: (1-coef[Ph[i],1])*t[Ph[i],2]-M*(1-del[i])<=(1-coef[Ph[i],1])*delT2[i];  
 s.t. RVehD20{i in I: (active_arr[i]>0)}: (1-coef[Ph[i],1])*t[Ph[i],2]+M*(1-del[i])>=(1-coef[Ph[i],1])*delT2[i];  
 s.t. RVehD21{i in I: (active_arr[i]>0)}: (1-coef[Ph[i],1])*delT2[i]<=M*del[i];  
  s.t. RVehD22{i in I: (active_arr[i]>0)}: coef[Ph[i],1]*(t[Ph[i],1]-M*(1-delZe[i]))<=delZeT1[i];  
 s.t. RVehD23{i in I: (active_arr[i]>0)}: t[Ph[i],1]+M*(1-delZe[i])>=coef[Ph[i],1]*delZeT1[i];  
 s.t. RVehD24{i in I: (active_arr[i]>0)}: coef[Ph[i],1]*delZeT1[i]<=M*delZe[i];  
 s.t. RVehD25{i in I: (active_arr[i]>0)}: (1-coef[Ph[i],1])*(t[Ph[i],2]-M*(1-delZe[i]))<=delZeT2[i];  
 s.t. RVehD26{i in I: (active_arr[i]>0)}: t[Ph[i],2]+M*(1-delZe[i])>=(1-coef[Ph[i],1])*delZeT2[i];  
 s.t. RVehD27{i in I: (active_arr[i]>0)}: (1-coef[Ph[i],1])*delZeT2[i]<=M*delZe[i];  
 s.t. RVehD28{i in I: (active_arr[i]>0)}: delZe[i]<=del[i];  
 s.t. RVehD29{i in I: (active_arr[i]>0)}: delZe[i]<=ze[i];  
 s.t. RVehD30{i in I: (active_arr[i]>0)}: delZe[i]>=del[i]+ze[i]-1;  
 s.t. RVehD31{i in I: (active_arr[i]>0)}: ((1-coef[Ph[i],1])*t[Ph[i],2] + coef[Ph[i],1]*t[Ph[i],1] - Ratio[i])<=M*ze[i];  
 s.t. RVehD32{i in I: (active_arr[i]>0)}: ((1-coef[Ph[i],1])*t[Ph[i],2] + coef[Ph[i],1]*t[Ph[i],1] - Ratio[i])>=M*(ze[i]-1);  
 s.t. RVehD33{i in I: (active_arr[i]>0)}: s*(((1-coef[Ph[i],1])*t[Ph[i],2] + coef[Ph[i],1]*t[Ph[i],1]) + L0[Ln[i]])/(s-qSp)- Ratio[i]<=M*del[i];  
 s.t. RVehD34{i in I: (active_arr[i]>0)}: s*(((1-coef[Ph[i],1])*t[Ph[i],2] + coef[Ph[i],1]*t[Ph[i],1]) + L0[Ln[i]])/(s-qSp)- Ratio[i]>=M*(del[i]-1);  
 s.t. RVehD35{i in I: (active_arr[i]>0)}: qq[i]<=M*qIndic[i];  
 s.t. RVehD36{i in I: (active_arr[i]>0)}: qq[i]>=M*(qIndic[i]-1);  
 s.t. RVehD37{i in I: (active_arr[i]>0)}: qq[i]-M*(1-qIndic[i])<=q[i];  
 s.t. RVehD38{i in I: (active_arr[i]>0)}: qq[i]+M*(1-qIndic[i])>=q[i];  
 s.t. RVehD39{i in I: (active_arr[i]>0)}: q[i]<=M*qIndic[i];  
   minimize delay:  TotRegVehDel+  PriorityDelay;     
  
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
