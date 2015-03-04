function m=YesOrNoPrompt(msg)

    while(1)
        %%your code here
        m=input([msg 'y/n :'],'s');
        if m=='y' || m=='n'
            break
        end
    end
end