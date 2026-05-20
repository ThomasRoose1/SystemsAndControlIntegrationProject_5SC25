function u = multisine(f_1,f_2,fs,N,A) %returns one periode
    if(nargin < 5)
        A = 1; %default amplitude
    end
    if f_1 < 0 || f_2 < 0
        error("Frequencies must be positive: f_min = %d, f_max = %d",f_1,f_2);
    end
    fmi = min(f_1,f_2);
    fma = max(f_1,f_2);
    f_min = fmi;
    f_max = fma;

    if f_max > fs/2
        warning("f_max > nyquist freq, f_max = fs/2");
        f_max = fs/2;
    end
    
    df = fs / N; %Frequency resolution
    K = (N-(~isEven(N)))/2 %Freq grid in K samples -> nyquist for even or odd N
    
    f = (0:K)*df %full freq grid

    k = ceil(f_min/df) +(ceil(f_min/df) == 0) : floor(f_max/df) % Frequency domain vector
    
    phi = rand(size(k))*2*pi; %create random phases
    ck = exp(1j*phi); %create random phase coeficients

    cn = zeros(1,N);
    cn(k) = ck; %set random phase in freq domain
    
    cn(1) = 0; %dc gain
    % cn(end) = 0; %nyquist
    u = 2*real(ifft(cn));


    %normilize amplitude to A
    u = 1/max(abs(u))*u*A;
end