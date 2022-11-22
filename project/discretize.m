function [ F, H ] = discretize( A, B, T )

    F = expm(A * T);

    Int_Phi = T * eye(length(F(:,1)));
    Delta_Int_Phi = T;
    count = 2;
    while 1
        Delta_Int_Phi = Delta_Int_Phi * A * T / count;
        Int_Phi = Int_Phi + Delta_Int_Phi;
        count = count + 1;
        if all(abs(Delta_Int_Phi(:)) <= eps), break; end
    end

    H = Int_Phi * B;

end