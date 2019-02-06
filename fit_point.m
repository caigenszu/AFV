function [A,B,C,D]=fit_point(s)
[A,B,C,D]=PlaneFitting(s);
E=[A B C];
[n,~]=size(s);
d=abs(vpa(sum(bsxfun(@times,E,s(1:n,:)),2)-D));
do=mean(d);
od=sqrt((d-do)'*(d-do)/(n-1));
L=double(d>3*od);
while max(L)~=0 
   t=find(L~=0);
   s(t,:)=[];
   [A,B,C,D]=PlaneFitting(s);
   E=[A B C];
   [n,~]=size(s);
   d=abs(vpa(sum(bsxfun(@times,E,s(1:n,:)),2)-D));
   do=mean(d);
   od=sqrt((d-do)'*(d-do)/(n-1));
   L=double(d>3*od);
if max(L)<1
     break
end
end
