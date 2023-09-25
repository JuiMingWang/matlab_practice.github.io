A = [1:2000];
A(1:999) = -1;
%找出prime number
for i = [1000:2000]
    for j = [ 2:ceil( sqrt(i) ) ]
      if rem(i,j) == 0  %表示i有因數，因此他不是質數
        A(i) = 0;%改為0用來表示他是質數
        break    %若是質數將不用繼續檢查
      else
        continue
      end
    end
end
%挑出已找到的prime number
k = 1;
B = [];
for x =[1000:2000]
    if A(x) ~= 0
        B(k) = A(x);
        k = k+1;
    else
        continue
    end
end
disp(B)