const float control_dt = 0.02;

const float vel[] = {-0.08882369779383321, -0.17017313971027806, -0.16089229253757253, -0.1343438833449381, -0.08284556213195292, -0.021108349639361916, 0.05417724121387801, 0.1213497291372327, 0.19644357911829838, 0.27980571825769746, 0.3192906111753431, 0.3675174331136513, 0.42601966491612375, 0.4617637822736678, 0.4851965343882127, 0.4908863024455856, 0.482029785589487, 0.4616406847652158, 0.43872283957567726, 0.4105743156171017, 0.37903469626854464, 0.35481800338678526, 0.3245917757761261, 0.2959469272965528, 0.25062659708511514, 0.19163982241095873, 0.1358823034358903, 0.06776236619655931, -0.007463963882347203, -0.09103878167839151, -0.15008734099149343, -0.21602929505372295, -0.28722723501666786, -0.3616747757319475, -0.4117828377727951, -0.4334547275821335, -0.44621332881646303, -0.46856203920324935, -0.506396682729074, -0.5392926180504188, -0.5466037842644482, -0.5454945604141623, -0.5230350287476626, -0.49054513020479973, -0.4492594772056204, -0.4024100250814282, -0.3440516096039831, -0.27431727712556475, -0.20316879243298774, -0.15149206141955518, -0.10861672638425006, -0.05678657930776823, 0.0019279944026108532, 0.06513253175086865, 0.1596337946173121, 0.25289503596455104, 0.35054285011801317, 0.40436288614735766, 0.45175317851093955, 0.49256124109303384, 0.5272057449624651, 0.5464831653389023, 0.5602056790748039, 0.5514483180489863, 0.5340434777578938, 0.5073292118035073, 0.47019551505955104, 0.42106933294786203, 0.35814471747306276, 0.29779151398151327, 0.23610767048525366, 0.1664559604683776, 0.0860465525875842, 0.02855100589787935, -0.03436084776839219, -0.09942460609435673, -0.19451465565757298, -0.289437387223414, -0.3812592977155071, -0.4825242176069726, -0.552831111803901, -0.6143811108061751, -0.6383056243718093, -0.6552358874670021, -0.6662543015156638, -0.672826313502528, -0.6757657349333633, -0.6751143812824756, -0.6701009300106774, -0.6591892133110193, -0.6402434313056614, -0.6109197332471823, -0.5689104374725689, -0.5128739291743952, -0.4488811119476494, -0.3778342039974561, -0.29962566482314906, -0.21699895803885105, -0.16409415216838016, -0.10033972617135273, -0.013291109459848759, 0.06696905633830749, 0.15492554213582757, 0.23651817835997074, 0.30873318033685004, 0.3719561321707865, 0.42872241369054787, 0.48149420883640387, 0.5490017467532172, 0.6176070805543424, 0.6871142144391303, 0.7508127948718856, 0.7955775244406593, 0.8105750944914923, 0.8049534610810604, 0.745297040123975, 0.6749652994418999, 0.6538234578139361, 0.6151717618031314, 0.5779739386291227, 0.5349710814331157, 0.45747539565506756, 0.365107054285096, 0.29205810108018965, 0.19438344445668862, 0.10410133498044197, 0.02046902045297195, -0.07068174172609827, -0.147723074244205, -0.21635346773396058, -0.2932774641828904, -0.3532746212583853, -0.4119738359682272, -0.4706530947307105, -0.5299579250269106, -0.5897159606143059, -0.6494462133680852, -0.721159069133798, -0.7923882515187097, -0.84150967168367, -0.863473334210659, -0.8699383163159731, -0.8481257938829365, -0.7758002916237244, -0.6980888080147362, -0.6489246442280932, -0.5767385213339796, -0.4953892779317826, -0.38089380381830096, -0.2560068877631912, -0.136255314751679, -0.02329719021749807, 0.08001547457314141, 0.151077783371314, 0.2178712140693047, 0.26179364070867206, 0.3046598623859432, 0.33002003507069133, 0.3562944902693967, 0.38384474146678277, 0.4310256970967031, 0.514197797074578, 0.6105314801866027, 0.7019243182721333, 0.7814747560669714, 0.8430618421047203, 0.8910637930338746, 0.9240655423131378, 0.9118008786621755, 0.8935394542687103, 0.8632266409535245, 0.765690604030253, 0.6406788001464907, 0.5007433991086513, 0.357292390636507, 0.20776418517291922, 0.07208482637161855, -0.03287880894885745, -0.1035750855143595, -0.15658653043883364, -0.20692692510070204, -0.25839435140069794, -0.2923023841258213, -0.33030082100960956, -0.37176740255342444, -0.4302516871285322, -0.4949993290692484, -0.5801806963975097, -0.6788999901605496, -0.763981109717205, -0.8466559651921063, -0.9098694384425499, -0.9643966170394329, -0.9951809411032583, -0.9736416921587494, -0.9110970209960703, -0.8079969107044814, -0.6714359618189346, -0.53137342848242, -0.38306339545919, -0.22706966623710484, -0.08948263019301791, 0.019322868420247746, 0.10319645229352677, 0.1428966293020526, 0.1634572994095687, 0.17339797368144277, 0.19356106985521304, 0.221109085551424, 0.25261053929087196, 0.2554753670861752, 0.2613595291600458, 0.34899523695528645, 0.4676689070018249, 0.6145583663749349, 0.7655299490698987, 0.8875927812292163, 0.9924289677482213, 1.0558694341288428, 1.0796559430461126, 1.0733821157279164, 1.0119416629013702, 0.9177277314296489, 0.7672949128521447, 0.5805600765956733, 0.38652115709952894, 0.2054949053258578, 0.05781363435631648, -0.050996619646573316, -0.11699514882138451, -0.15245781916882528, -0.14208590057270631, -0.09474321838945032, -0.024631942103935822, 0.05975234825317799, 0.13170570927211322, 0.1657483536388581, 0.1566687972554961, 0.08752797867867501, -0.008374785289993802, -0.1319405937537078, -0.33659573288884304, -0.5465593072547469, -0.7255222130156017, -0.8632467902821058, -0.9552077031644609, -0.99301288864849, -1.0038968083316964, -0.9951371859203185, -0.9243813781445566, -0.7837409310678064, -0.5900100430454077, -0.3518986661019866, -0.15969630560183157, -0.003440381297031947, 0.12226679536298701, 0.19102212785506878, 0.20270481798288198, 0.15826726573528715, 0.071995522067819, -0.052966684502647965, -0.20248762629588368, -0.3556395458317728, -0.5180824569172372, -0.6738578916177486, -0.7427439487432502, -0.7232272573529958, -0.6111917907418628, -0.42362004675455334, -0.15096873709729808, 0.11676520372038612, 0.3850557971236641, 0.6330954048505374, 0.8362617159346452, 0.9813409048830346, 1.0651229400379314, 1.0868044224037705, 0.9888907249095578, 0.7956097153155326, 0.5219789362476388, 0.17628543558155807, -0.0680594987342984, -0.24718627917844527, -0.36678711911375717, -0.4265819835242485, -0.4438106761332009, -0.4188756261368377, -0.34824463117037696, -0.25036669089715813, -0.12183741459638942, 0.03542089740921561, 0.2234513885610826, 0.4354803754737827, 0.6632997961096244, 0.7216207631789844, 0.5601770401487083, 0.3935346387561424, 0.271123532858289, 0.13113856231823073, -0.018200139409843552};