function[sample]=CalcSample(b)
A=(1-(-1)).*rand(12,1)-1;
sample=b*sum(A)/6;
end