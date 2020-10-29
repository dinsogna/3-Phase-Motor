close;
Array=[out.Position_Output{2}.Values.Time,out.Position_Output{2}.Values.Data,out.Position_Output{1}.Values.Time,out.Position_Output{1}.Values.Data];   
writematrix(Array,"CLDT_20hz_Sine.csv")
plot(Array(:,1),Array(:,2),Array(:,3),Array(:,4))
% writematrix(out.Open_Loop,"OLDT_07801_3.csv")
% plot(out.Open_Loop(:,1),out.Open_Loop(:,2))