function [sig]=timsim(counter3,counter2,counter4)
sig=[];
for(a=1:1161216)
	counter3+=1;
	if(counter3<5983)
		counter2+=1;
		if(counter2<5921)
			counter4+=1;
			if(counter4==5983)
				counter4=0;
			endif
		else
			if(counter2==5952)
				counter2=0;
			endif
		endif
	else
		if(counter3==6048)
			counter3=0;
		endif
	endif
	sig(a,:)=[counter3,counter2,counter4];
	printf("%f   \r",a/1161216);
	fflush(stdout);
endfor
endfunction  
