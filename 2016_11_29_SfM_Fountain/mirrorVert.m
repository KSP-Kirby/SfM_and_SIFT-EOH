function [ imgOut ] = mirrorVert( imgIn )
%creates left-right mirror image of imput image

imgOut = zeros(size(imgIn));

    for i = 1:size(imgIn,1)
        imgOut(size(imgIn,1)-i+1,:,:) = imgIn(i,:,:);
    end
end

