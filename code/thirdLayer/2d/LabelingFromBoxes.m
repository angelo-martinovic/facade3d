function output = LabelingFromBoxes(best_boxes,full,imageSize,hyperParameters)

output = zeros(imageSize);
% Objects
   wins = (best_boxes(:,best_boxes(5,:) == hyperParameters.winClass));
   output = FillOutput(output,wins,hyperParameters.winClass,full);
   
   if isfield(hyperParameters,'balcClass')
       balcs = (best_boxes(:,best_boxes(5,:) == hyperParameters.balcClass));
       output = FillOutput(output,balcs,hyperParameters.balcClass,full);
   end
   
   if isfield(hyperParameters,'doorClass')
       doors = (best_boxes(:,best_boxes(5,:) == hyperParameters.doorClass));
       output = FillOutput(output,doors,hyperParameters.doorClass,full);
   end
   
end

function output = FillOutput(output,objects,class,full)

   ss = randperm(size(objects,2));
   for i=1:size(objects,2)
       w = objects(:,i);
       w(1:4) = round(w(1:4));
       if full
        output(w(2):w(4), w(1):w(3)) = class;
       else
        output(w(2):w(4), w(1)) = w(6);%ss(i);
        output(w(2):w(4), w(3)) = w(6);%ss(i);
        output(w(2), w(1):w(3)) = w(6);%ss(i);
        output(w(4), w(1):w(3)) = w(6);%ss(i);
       end
   end
   
end